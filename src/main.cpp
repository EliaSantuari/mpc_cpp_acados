#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <chrono>

#include "config.h"
#include "gait_planner.h"
#include "obstacle_manager.h"
#include "data_logger.h"

#include "acados_solver_lipm_opti_pessi.h"

// Helper
double norm2D(const std::array<double, 2>& v) {
    return std::sqrt(v[0]*v[0] + v[1]*v[1]);
}
double dot2D(const std::array<double, 2>& v1, const std::array<double, 2>& v2) {
    return v1[0]*v2[0] + v1[1]*v2[1];
}


std::array<double, 4> random_obs_gen(){
    static std::random_device rd;
    static std::mt19937 gen = std::mt19937(rd());

    std::uniform_real_distribution<double> pos_init_x(1.0, 9.0);
    std::uniform_real_distribution<double> pos_init_y(-2.0, 2.0);
    std::uniform_real_distribution<double> r_obs(0.1, 0.4);
    std::uniform_real_distribution<double> speed(0.1, 0.5);
    
    return {pos_init_x(gen), pos_init_y(gen), r_obs(gen), speed(gen)};
}



int main() {
    // Total time of main
    auto start_time_tot = std::chrono::high_resolution_clock::now();

    Config::load("../config.yaml");

    std::cout << "--- Starting Simulation MPC in C++ ---" << std::endl;

    // Initialization of acados using the "python model"
    lipm_opti_pessi_solver_capsule* capsule = lipm_opti_pessi_acados_create_capsule();
    if (lipm_opti_pessi_acados_create(capsule) != 0) {
        std::cerr << "Error in initializing acados!" << std::endl;
        return 1;
    }

    // Instantiate gait_planner
    GaitPlanner gait_planner(Config::off_x, Config::off_y, Config::step_per_phase);

    // Instantiate obs_manager
    size_t N_obs_max = Config::N_obs_max;
    ObstacleManager obs_manager(N_obs_max);
    


    // Populate manager with obstacles
    for(size_t i = 0; i < N_obs_max; ++i){
        if(Config::obs_type == "random"){
            std::array<double, 4> bounds = {1.0, 9.0, -2.0, 2.0};
            auto [x, y, r, speed] = random_obs_gen();
            std::array<double, 2> start_pos = {x, y};
            std::cout << x << " " << y << " " << r << " " << speed << std::endl;
            obs_manager.add_obstacle(std::make_unique<RandomObstacle>(start_pos, r, speed, bounds, Config::y_dot_max));
        }else if(Config::obs_type == "static"){
            obs_manager.add_obstacle(std::make_unique<StaticObstacle>(Config::obs_pos_init, Config::r_obs, Config::y_dot_max));
        }else if(Config::obs_type == "dynamic"){
            obs_manager.add_obstacle(std::make_unique<DynamicObstacle>(Config::top_pos_dyn, Config::bot_pos_dyn, Config::r_obs, Config::speed, Config::y_dot_max));
        }else if(Config::obs_type == "adversarial"){
            obs_manager.add_obstacle(std::make_unique<AdversarialObstacle>(Config::obs_pos_init, Config::r_obs, Config::speed, Config::y_dot_max));
        }else if(Config::obs_type == "circular"){
            obs_manager.add_obstacle(std::make_unique<CircularObstacle>(Config::obs_pos_init, Config::obs_center, Config::r_obs, Config::speed, Config::y_dot_max));
        }
    }


    double dt_chosen = Config::dt_max; // Initialization


    // Intstantiate logger
    DataLogger logger(Config::sim_steps, Config::N_horizon, Config::N_obs_max);

    
    // State and control vectors initialization
    constexpr int NX = LIPM_OPTI_PESSI_NX; 
    constexpr int NU = LIPM_OPTI_PESSI_NU; 
    constexpr int NP = LIPM_OPTI_PESSI_NP;  
    int n_u_branch = 8 + (3 * N_obs_max);

    std::array<double, 11> X_sim = {0};
    X_sim[0] = Config::x_init[0];
    X_sim[1] = Config::x_init[1];
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
    u_init[n_u_branch + 7] = Config::dt_max;
    std::vector<std::array<double, NU>> U_sol_t_prec(Config::N_horizon, u_init);

    

    double t_global = 0.0;
    int reached_target_flag = 0;

    // Storing computation time
    std::vector<double> time_hist;
    time_hist.reserve(Config::sim_steps); // allocate memory
    std::array<double, 2> obs_pos;


    


    // ------ MAIN LOOP ------
    for (int step = 0; step < Config::sim_steps; ++step) {
        // Position of the obstacle
        std::array<double, 2> robot_pos = {X_sim[0], X_sim[1]};
        obs_manager.update_all(dt_chosen, t_global, robot_pos);
        auto active_obstacles = obs_manager.get_active_obstacles();
        
        
        // Set time to zero for this loop
        X_sim[10] = 0.0; 
        std::copy(X_sim.begin(), X_sim.end(), X_aug.begin());
        std::copy(X_sim.begin(), X_sim.end(), X_aug.begin() + 11);
        
        // Set the initial guess as X_aug
        ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, 0, "lbx", X_aug.data());
        ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out, 0, "ubx", X_aug.data());


        // Local target
        double dir_to_target_x = Config::c_target[0] - X_sim[0];
        double dir_to_target_y = Config::c_target[1] - X_sim[1];
        std::array<double, 2> dir_to_target = {dir_to_target_x, dir_to_target_y};
        double dist_to_final = norm2D(dir_to_target);

        double max_lookahead = 1.5;
        double local_target_x;
        double local_target_y;
        if(dist_to_final > max_lookahead){
            local_target_x = X_sim[0] + (dir_to_target_x / dist_to_final) * max_lookahead;
            local_target_y = X_sim[1] + (dir_to_target_y / dist_to_final) * max_lookahead;
        }else{
            local_target_x = Config::c_target[0];
            local_target_y = Config::c_target[1];
        }


        // Setting target pointing to model in c_generated_code
        double yref[LIPM_OPTI_PESSI_NY] = {0};
        for(int k = 0; k < Config::N_horizon; ++k){
            yref[0] = local_target_x;
            yref[1] = local_target_y;
            yref[17] = local_target_x;
            yref[18] = local_target_y;

            ocp_nlp_cost_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, k, "yref", yref);
        }
        double yref_e[LIPM_OPTI_PESSI_NYN] = {0};
        yref_e[0] = local_target_x;
        yref_e[1] = local_target_y;
        yref_e[6] = local_target_x;
        yref_e[7] = local_target_y;
        ocp_nlp_cost_model_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, Config::N_horizon, "yref", yref_e);



        // ---- SET PARAMETERS and WARM START -----
        // Horizon of the gait
        auto gait_horizon = gait_planner.get_gait_horizon(step, Config::N_horizon);

        for (int k = 0; k < Config::N_horizon+1; ++k){
            auto gait_k = gait_horizon[k];

            auto off0 = gait_planner.get_hip_offset(gait_k.first);  
            auto off1 = gait_planner.get_hip_offset(gait_k.second);

            std::vector<double> p_val(NP, 0.0);
            p_val[0] = off0[0]; p_val[1] = off0[1];
            p_val[2] = off1[0]; p_val[3] = off1[1];

            for(size_t i = 0; i < N_obs_max; ++i){
                int idx_p = 4 + (4 * i);
                if(i < active_obstacles.size()){
                    p_val[idx_p] = active_obstacles[i].pos[0];
                    p_val[idx_p + 1] = active_obstacles[i].pos[1];
                    p_val[idx_p + 2] = active_obstacles[i].r_obs + 0.1;
                    p_val[idx_p + 3] = active_obstacles[i].y_dot_max;
                }else{
                    // GHOST OBSTACLES
                    p_val[idx_p] = 15.0 + (2.0 * i);
                    p_val[idx_p + 1] = 15.0;
                    p_val[idx_p + 2] = 0.0;
                    p_val[idx_p + 3] = 0.0;
                }
            }
            // Set params
            lipm_opti_pessi_acados_update_params(capsule, k, p_val.data(), NP);
        }


        // Lambda helper to comput the hyperplane (ax, ay, b) given the obstacle and robot points
        auto calc_hyperplane_for_obs = [](const std::array<double, 2>& obs_p, 
                                        const std::array<double, 2>& com,
                                        const std::array<double, 2>& hip0,
                                        const std::array<double, 2>& hip1) -> std::array<double, 3>{
            double d_com = norm2D({obs_p[0]-com[0], obs_p[1]-com[1]});
            double d_h0 = norm2D({obs_p[0]-hip0[0], obs_p[1]-hip0[1]});
            double d_h1 = norm2D({obs_p[0]-hip1[0], obs_p[1]-hip1[1]});

            double min_d = std::min({d_com, d_h0, d_h1});
            std::array<double, 2> closest = com;
            if(min_d == d_h0) closest = hip0;
            else if(min_d == d_h1) closest = hip1;

            std::array<double, 2> dir = {obs_p[0] - closest[0], obs_p[1] - closest[1]};
            double dist = norm2D(dir) + 1e-5;
            std::array<double, 2> a = {dir[0]/dist, dir[1]/dist};
            std::array<double, 2> mid = {(closest[0]+obs_p[0])/2.0, (closest[1]+obs_p[1])/2.0};
            double b = -dot2D(a, mid);

            return {a[0], a[1], b};
        };


        for (int k = 0; k < Config::N_horizon; ++k) {
            auto gait_k = gait_horizon[k];
            
    
            // Indeces for shifts: last step repeat last value
            int idx_shift_u = (k < Config::N_horizon - 1) ? k + 1 : k;
            int idx_shift_x = k + 1;

            const auto& x_prev = X_sol_t_prec[idx_shift_x];
            const auto& u_prev = U_sol_t_prec[idx_shift_u];

            // Optimistic and pessimistic
            std::array<double, 2> com_op = {x_prev[0], x_prev[1]};
            std::array<double, 2> com_pe = {x_prev[11], x_prev[12]};

            // Guess of hips position
            auto hips_op = gait_planner.compute_hip_positions({com_op[0], com_op[1], x_prev[2]}, gait_k);
            auto hips_pe = gait_planner.compute_hip_positions({com_pe[0], com_pe[1], x_prev[13]}, gait_k);
            std::array<double, 2> h0_op = {hips_op[0], hips_op[1]};
            std::array<double, 2> h1_op = {hips_op[2], hips_op[3]};
            std::array<double, 2> h0_pe = {hips_pe[0], hips_pe[1]};
            std::array<double, 2> h1_pe = {hips_pe[2], hips_pe[3]};

            // Base vector for u_guess (only first 8 elements)
            std::vector<double> u_guess_op = {hips_op[0], hips_op[1], hips_op[2], hips_op[3], 0.5, 0.0, 0.0, u_prev[7]};
            std::vector<double> u_guess_pe = {hips_pe[0], hips_pe[1], hips_pe[2], hips_pe[3], 0.5, 0.0, 0.0, u_prev[n_u_branch + 7]};

            // Add hyperplanes for each obstacle
            for(size_t i = 0; i < N_obs_max; ++i){
                std::array<double, 2> obs_pos_k;
                if(i < active_obstacles.size()) obs_pos_k = active_obstacles[i].pos;
                else obs_pos_k = {15.0 + 2.0 * i, 15.0};

                auto hp_op = calc_hyperplane_for_obs(obs_pos_k, com_op, h0_op, h1_op);
                auto hp_pe = calc_hyperplane_for_obs(obs_pos_k, com_pe, h0_pe, h1_pe);

                u_guess_op.insert(u_guess_op.end(), hp_op.begin(), hp_op.end());
                u_guess_pe.insert(u_guess_pe.end(), hp_pe.begin(), hp_pe.end());
            }

            // Unify the two branches
            std::vector<double> u_guess_tot;
            if (k == 0){
                // Fisrt step they should be the same - pessimistic
                u_guess_tot = u_guess_pe;
                u_guess_tot.insert(u_guess_tot.end(), u_guess_pe.begin(), u_guess_pe.end());
            }else {
                u_guess_tot = u_guess_op;
                u_guess_tot.insert(u_guess_tot.end(), u_guess_pe.begin(), u_guess_pe.end());
            }


            // Insert the guess in acados
            ocp_nlp_out_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, capsule->nlp_in, k, "u", u_guess_tot.data());

            // Construction of X_guess
            std::array<double, 3> state_op = {x_prev[0], x_prev[1], x_prev[2]};
            std::array<double, 3> state_pe = {x_prev[11], x_prev[12], x_prev[13]};
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
                x_guess[21] = u_prev[n_u_branch + 7]; // Propagation of previous dt

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
        dt_chosen = u_opt[7];
        t_global += dt_chosen;


        // Extraction of the opti-pessi predictions
        std::vector<std::array<double, 2>> pred_op(Config::N_horizon);
        std::vector<std::array<double, 2>> pred_pe(Config::N_horizon);
        for(int k = 0; k < Config::N_horizon; ++k){
            pred_op[k] = {X_sol_t_prec[k+1][0], X_sol_t_prec[k+1][1]};
            pred_pe[k] = {X_sol_t_prec[k+1][11], X_sol_t_prec[k+1][12]};
        }

        std::array<double, 11> u_apply;
        std::copy(u_opt.begin(), u_opt.begin() + 11, u_apply.begin());

        logger.log_step(solve_time, X_sim, u_apply, active_obstacles, pred_op, pred_pe);

        
        
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
        std::cout << "Average solving frequency: " << 1.0 / mean_t * 1000.0 << " Hz" << std::endl; 
        std::cout << "------------------------------" << std::endl;
    }


    // Total time of main
    auto end_time_tot = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_tot = end_time_tot - start_time_tot;
        double solve_time_tot = elapsed_tot.count()*1000; // In milli seconds

    std::cout << "Total program time: " << solve_time_tot << " ms" << std::endl;

    
    // Deallocate 
    lipm_opti_pessi_acados_free(capsule);
    lipm_opti_pessi_acados_free_capsule(capsule);

    // Save data
    logger.save_to_csv("mpc_simulation_data.csv");

    return 0;
}