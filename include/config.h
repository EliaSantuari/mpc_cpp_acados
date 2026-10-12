
#pragma once
#include<numbers>
#include<vector>
#include<string>


namespace Config{
    // ROBOT CONFIGURATION
    constexpr double h_com = 0.38;          // m
    constexpr double g = 9.81;              // m/s^2
    constexpr double m = 24.24;             // kg
    constexpr double Iz = 1.048;            // kg/m^2
    constexpr double off_x = 0.2407;        // m
    constexpr double off_y = 0.134;         // m
    constexpr double mu = 0.8;              // adimensional
    constexpr double max_ext_sq = 0.3*0.3;  // m^2: leg limit extension 
    constexpr double x_init_x = 0.0;       // m
    constexpr double x_init_y = 0.0;       // m
    constexpr double theta_init = 0.0;      // rad

    // LIMITS
    constexpr double dt_min = 0.1;          // sec
    constexpr double dt_max = 0.25;         // sec
    constexpr double theta_dot_max = 0.6;   // rad/sec
    constexpr double v_max_x = 1.5;         // m/s
    constexpr double v_max_y = 0.45;        // m/s
    constexpr double alpha_min = 0.1;       // 
    constexpr double alpha_max = 0.9;       // 

    // OCP WEIGHTS
    constexpr double tracking_xy = 5.0;     // Tracking x, y
    constexpr double vel_alignment = 10.0;  // Velocity alignment - penalize lateral walks
    constexpr double vel_xy = 5.0;          // Velocities x,y
    constexpr double yaw_rate = 1;          // Yaw Rate - penalize fast rotations
    constexpr double anti_skating = 10.0;   // Curr foot and next foot
    constexpr double posture = 1e4;         // Keep hips above feet
    constexpr double alpha_weight = 1;      // Alpha
    constexpr double dt_weight = 1e-3;      // dt


    // SIMULATION
    constexpr int N_horizon = 7;
    constexpr int sim_steps = 200;
    constexpr int step_per_phase = 1;
    inline const std::vector<double> c_target = {1.0, 0.2};
 
    
    // OBSTACLES
    inline const std::vector<double> obs_pos_init = {0.1, 3.0};
    constexpr double r_obs = 0.2;           // Obstacle radius
    constexpr double speed = 0.5;           // Obs speed
    constexpr double y_dot_max = 1.0;       // Speed against which the pessimistic is robust
    inline const std::vector<double> top_pos_dyn = {0.1, 1.0};
    inline const std::vector<double> bot_pos_dyn = {0.1, -1.0};
    inline const std::string obs_type = "static";
}