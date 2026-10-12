#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <chrono>

#include "config.h"
#include "gait_planner.h"

#include "acados_solver_lipm_opti_pessi.h"

// Helper
double norm2D(const std::array<double, 2>& v) {
    return std::sqrt(v[0]*v[0] + v[1]*v[1]);
}
double dot2D(const std::array<double, 2>& v1, const std::array<double, 2>& v2) {
    return v1[0]*v2[0] + v1[1]*v2[1];
}

int main() {
    std::cout << "--- Starting Simulation MPC in C++ ---" << std::endl;

    // Initialization of acados using the "python model"
    lipm_opti_pessi_solver_capsule* capsule = lipm_opti_pessi_acados_create_capsule();
    if (lipm_opti_pessi_acados_create(capsule) != 0) {
        std::cerr << "Error in initializing acados!" << std::endl;
        return 1;
    }

    // Instantiate gait_planner
    GaitPlanner gait_planner(Config::off_x, Config::off_y, Config::step_per_phase);

    
    // State and control vectors initialization
    constexpr int NX = 22; 
    constexpr int NU = 22; 
    constexpr int NP = 8;  

    std::array<double, 11> X_sim = {0};
    X_sim[0] = Config::x_init_x;
    X_sim[1] = Config::x_init_y;
    X_sim[2] = Config::theta_init;
    // Initialization of the gait
    auto init_gait = gait_planner.get_gait_horizon(0, Config::N_horizon)[0];
    auto initial_hips = gait_planner.compute_hip_positions({X_sim[0], X_sim[1], X_sim[2]}, init_gait);
    // Initial position of the selected feet exactly underneath the hips
    X_sim[6] = initial_hips[0]; X_sim[7] = initial_hips[1];
    X_sim[8] = initial_hips[2]; X_sim[9] = initial_hips[3];


    // Concatenate for X_aug
    std::array<double, NX> X_aug = {0};
    std::copy(X_sim.begin(), X_sim.end(), X_aug.begin());
    std::copy(X_sim.begin(), X_sim.end(), X_aug.begin() + 11);

    // Initialization lists for warm start
    std::vector<std::array<double, NX>> X_sol_t_prec(Config::N_horizon + 1, X_aug);
    
    std::array<double, NU> u_init = {0};
    u_init[7] = Config::dt_max;
    u_init[18] = Config::dt_max;
    std::vector<std::array<double, NU>> U_sol_t_prec(Config::N_horizon, u_init);

    // Log of values
    std::ofstream log_file("mpc_log.csv");
    log_file << "step,x,y,theta,vx,vy\n"; 

    double t_global = 0.0;
    int reached_target_flag = 0;

    // Storing computation time
    std::vector<double> time_hist;
    time_hist.reserve(Config::sim_steps); // allocate memory

    // ------ MAIN LOOP ------
    for (int step = 0; step < Config::sim_steps; ++step) {
        // Position of the obstacle - STATIC
        std::array<double, 2> obs_pos = {Config::obs_pos_init[0], Config::obs_pos_init[1]};
        
        // Set time to zero for this loop
        X_sim[10] = 0.0; 
        std::copy(X_sim.begin(), X_sim.end(), X_aug.begin());
        std::copy(X_sim.begin(), X_sim.end(), X_aug.begin() + 11);
        
        // Set the initial guess as X_aug
        ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, 0, "lbx", X_aug.data());
        ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, 0, "ubx", X_aug.data());

        // ---- SET PARAMETERS and WARM START -----
        // Horizon of the gait
        auto gait_horizon = gait_planner.get_gait_horizon(step, Config::N_horizon);

        for (int k = 0; k < Config::N_horizon+1; ++k){
            auto gait_k = gait_horizon[k];

            auto off0 = gait_planner.get_hip_offset(gait_k.first);  
            auto off1 = gait_planner.get_hip_offset(gait_k.second);
            double r_dyn = Config::r_obs + 0.1;

            // Set params
            double p_val[NP] = {off0[0], off0[1], off1[0], off1[1], obs_pos[0], obs_pos[1], r_dyn, Config::y_dot_max};
            lipm_opti_pessi_acados_update_params(capsule, k, p_val, NP);

        }

        for (int k = 0; k < Config::N_horizon; ++k) {
            auto gait_k = gait_horizon[k];
            
    

            // Indeces for shifts: last step repeat last value
            int idx_shift_u = (k < Config::N_horizon - 1) ? k + 1 : k;
            int idx_shift_x = k + 1;

            const auto& x_prev = X_sol_t_prec[idx_shift_x];
            const auto& u_prev = U_sol_t_prec[idx_shift_u];

            // Optimistic and pessimistic
            std::array<double, 3> state_op = {x_prev[0], x_prev[1], x_prev[2]};
            std::array<double, 3> state_pe = {x_prev[11], x_prev[12], x_prev[13]};
            // Guess of hips position
            auto hips_guess_op = gait_planner.compute_hip_positions(state_op, gait_k);
            auto hips_guess_pe = gait_planner.compute_hip_positions(state_pe, gait_k);

            
            
            // Compute the closest point to obstacle
            auto calc_closest = [&](std::array<double, 2> pt, std::array<double, 4> hips) {
                std::array<double, 2> hip0 = {hips[0], hips[1]};
                std::array<double, 2> hip1 = {hips[2], hips[3]};
                double d_pt = norm2D({obs_pos[0]-pt[0], obs_pos[1]-pt[1]});
                double d_h0 = norm2D({obs_pos[0]-hip0[0], obs_pos[1]-hip0[1]});
                double d_h1 = norm2D({obs_pos[0]-hip1[0], obs_pos[1]-hip1[1]});
                double min_d = std::min({d_pt, d_h0, d_h1});
                if(min_d == d_h0) return hip0;
                if(min_d == d_h1) return hip1;
                return pt;
            };
            // Compute the hyperplane guess for each node
            auto calc_hyperplane = [&](std::array<double, 2> closest) {
                std::array<double, 2> dir = {obs_pos[0]-closest[0], obs_pos[1]-closest[1]};
                double dist = norm2D(dir) + 1e-5;
                std::array<double, 2> a = {dir[0]/dist, dir[1]/dist};
                std::array<double, 2> mid = {(closest[0]+obs_pos[0])/2.0, (closest[1]+obs_pos[1])/2.0};
                double b = -dot2D(a, mid);
                return std::make_pair(a, b);
            };
            
            // Return the closest point
            auto closest_op = calc_closest({state_op[0], state_op[1]}, hips_guess_op);
            auto closest_pe = calc_closest({state_pe[0], state_pe[1]}, hips_guess_pe);
            // Return the a,b pair
            auto hp_op = calc_hyperplane(closest_op);
            auto hp_pe = calc_hyperplane(closest_pe);


            // Fill up the u_guess vector
            std::array<double, NU> u_guess;
            auto fill_u = [](double* dst, std::array<double,4> hips, double dt, auto hp) {
                dst[0]=hips[0]; dst[1]=hips[1]; dst[2]=hips[2]; dst[3]=hips[3];
                dst[4]=0.5; dst[5]=0.0; dst[6]=0.0; dst[7]=dt;
                dst[8]=hp.first[0]; dst[9]=hp.first[1]; dst[10]=hp.second;
            };
            
            fill_u(&u_guess[0], hips_guess_op, u_prev[7], hp_op);
            fill_u(&u_guess[11], hips_guess_pe, u_prev[18], hp_pe);
            
            // First steps must be the same! We could use the pessimistic to be more conservative
            if (k == 0) std::copy(u_guess.begin()+11, u_guess.begin()+22, u_guess.begin());

            // Insert the guess in acados
            ocp_nlp_out_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, capsule->nlp_in, k, "u", u_guess.data());

            // Construction of X_guess
            std::array<double, NX> x_guess = {0};
            if (k > 0) {
                // Optimistic
                x_guess[0] = state_op[0]; x_guess[1] = state_op[1]; x_guess[2] = state_op[2];
                x_guess[3] = x_prev[3];   x_guess[4] = x_prev[4];   x_guess[5] = x_prev[5];
                x_guess[6] = x_prev[6];   x_guess[7] = x_prev[7];   
                x_guess[8] = x_prev[8];   x_guess[9] = x_prev[9];
                x_guess[10] = u_prev[7]; // Propagation of previous dt

                // Pessimistic
                x_guess[11] = state_pe[0]; x_guess[12] = state_pe[1]; x_guess[13] = state_pe[2];
                x_guess[14] = x_prev[14];  x_guess[15] = x_prev[15];  x_guess[16] = x_prev[16];
                x_guess[17] = x_prev[17];  x_guess[18] = x_prev[18];  
                x_guess[19] = x_prev[19];  x_guess[20] = x_prev[20];
                x_guess[21] = u_prev[18]; // Propagation of previous dt

                // Assign to acados
                ocp_nlp_out_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, capsule->nlp_in, k, "x", x_guess.data());
            }else {
                ocp_nlp_out_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, capsule->nlp_in, k, "x", X_sol_t_prec[0].data());
            }

            // Save the last x_guess for the last node of the horizon
            if (k == Config::N_horizon - 1) {
                ocp_nlp_out_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, capsule->nlp_in, Config::N_horizon, "x", x_guess.data());
            }
        }

        // Starting measuring time
        auto start_time = std::chrono::high_resolution_clock::now();

        // Call for the solver
        int status = lipm_opti_pessi_acados_solve(capsule);

        // End measuring time
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end_time - start_time;
        double solve_time = elapsed.count()*1000; // In seconds
        time_hist.push_back(solve_time);

        // Extract the controls and next state
        std::array<double, NU> u_opt;
        std::array<double, NX> x_next;
        ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, 0, "u", u_opt.data());
        ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, 1, "x", x_next.data());


        // Save the optimal state and control in X_sol_t_prec and U_sol_t_prec
        for (int k = 0; k < Config::N_horizon; ++k) {
            ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, k, "x", X_sol_t_prec[k].data());
            ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, k, "u", U_sol_t_prec[k].data());
        }
        ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, Config::N_horizon, "x", X_sol_t_prec.back().data());

        std::copy(x_next.begin() + 11, x_next.end(), X_sim.begin());

        // Save the optimal dt
        double dt_chosen = u_opt[7];
        t_global += dt_chosen;

        // Log values
        log_file << step << "," << X_sim[0] << "," << X_sim[1] << "," 
                 << X_sim[2] << "," << X_sim[3] << "," << X_sim[4] << "\n";
        
        // Print come data
        std::cout << "Step " << step << " | Pos: [" << X_sim[0] << ", " << X_sim[1] << "] | dt: " << dt_chosen*1000.0 << " ms | Comp.time: " << solve_time << " ms" << std::endl;
        
        // Control if the target is reached
        double dist_target = norm2D({X_sim[0] - Config::c_target[0], X_sim[1] - Config::c_target[1]});
        if (dist_target < 0.1) {
            reached_target_flag++;
            if (reached_target_flag == 5) {
                std::cout << "Target Reached!" << std::endl;
                break;
            }
        }
    }

    if (!time_hist.empty()) {
        double sum = 0.0;
        double max_t = time_hist[0];
        for (double t : time_hist) {
            sum += t;
            if (t > max_t) max_t = t;
        }
        double mean_t = sum / time_hist.size();

        std::cout << "\n--- Performance Statistics ---" << std::endl;
        std::cout << "Average solve time: " << mean_t << " ms" << std::endl;
        std::cout << "Maximum solve time: " << max_t  << " ms" << std::endl;
        std::cout << "------------------------------" << std::endl;
    }

    log_file.close();
    
    // Deallocate 
    lipm_opti_pessi_acados_free(capsule);
    lipm_opti_pessi_acados_free_capsule(capsule);

    return 0;
}