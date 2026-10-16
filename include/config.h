
#pragma once
#include<numbers>
#include<vector>
#include<string>


namespace Config{
    // ROBOT CONFIGURATION
    inline double h_com;          
    inline double g;              // m/s^2
    inline double m;             // kg
    inline double Iz;            // kg/m^2
    inline double off_x;        // m
    inline double off_y;         // m
    inline double mu;              // adimensional
    inline double max_ext_sq;           // m^2: leg limit extension 
    inline std::array<double, 2> x_init; // m
    inline double theta_init;      // rad

    // LIMITS
    inline double dt_min;          // sec
    inline double dt_max;         // sec
    inline double theta_dot_max;   // rad/sec
    inline double v_max_x;         // m/s
    inline double v_max_y;        // m/s
    inline double alpha_min;       // 
    inline double alpha_max;       // 

    // OCP WEIGHTS
    inline double tracking_xy;     // Tracking x, y
    inline double vel_alignment;  // Velocity alignment - penalize lateral walks
    inline double vel_xy;          // Velocities x,y
    inline double yaw_rate;          // Yaw Rate - penalize fast rotations
    inline double anti_skating;   // Curr foot and next foot
    inline double posture;         // Keep hips above feet
    inline double alpha_weight;      // Alpha
    inline double dt_weight;      // dt


    // SIMULATION
    inline int N_horizon;
    inline int sim_steps;
    inline int step_per_phase;
    inline std::array<double, 2> c_target;
 
    
    // OBSTACLES
    inline int N_obs_max;
    inline std::array<double, 2> obs_pos_init;
    inline double r_obs;           // Obstacle radius
    inline double speed;           // Obs speed
    inline double y_dot_max;       // Speed against which the pessimistic is robust
    inline std::array<double, 2> top_pos_dyn;
    inline std::array<double, 2> bot_pos_dyn;
    inline std::string obs_type; // static, dynamic, adversarial, circular
    inline std::array<double, 2> obs_center; // center for the circular movement

    // Function to read config.yml
    void load(const std::string& filepath);
}