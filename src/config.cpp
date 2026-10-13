#include "config.h"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <cstdlib>
// #include <string>
// #include <vector>


namespace Config{
    void load(const std::string& filepath){
        try{
            YAML::Node config = YAML::LoadFile(filepath);

            // ROBOT CONFIGURATION
            h_com = config["robot"]["h_com"].as<double>();
            g = config["robot"]["g"].as<double>();
            m = config["robot"]["m"].as<double>();
            Iz = config["robot"]["Iz"].as<double>();
            off_x = config["robot"]["off_x"].as<double>();
            off_y = config["robot"]["off_y"].as<double>();
            mu = config["robot"]["mu"].as<double>();
            max_ext_sq = config["robot"]["max_ext_sq"].as<double>();
            x_init = config["robot"]["x_init"].as<std::vector<double>>();
            theta_init = config["robot"]["theta_init"].as<double>();

            // LIMITS
            dt_min = config["limits"]["dt_min"].as<double>();
            dt_max = config["limits"]["dt_max"].as<double>();
            theta_dot_max = config["limits"]["theta_dot_max"].as<double>();
            v_max_x = config["limits"]["v_max_x"].as<double>();
            v_max_y = config["limits"]["v_max_y"].as<double>();
            alpha_min = config["limits"]["alpha_min"].as<double>();
            alpha_max = config["limits"]["alpha_max"].as<double>();

            // WEIGHTS - not used
            tracking_xy = config["ocp_weights"]["tracking_xy"].as<double>();
            vel_alignment = config["ocp_weights"]["vel_alignment"].as<double>();
            vel_xy = config["ocp_weights"]["vel_xy"].as<double>();
            yaw_rate = config["ocp_weights"]["yaw_rate"].as<double>();
            anti_skating = config["ocp_weights"]["anti_skating"].as<double>();
            posture = config["ocp_weights"]["posture"].as<double>();
            alpha_weight = config["ocp_weights"]["alpha_weight"].as<double>();
            dt_weight = config["ocp_weights"]["dt_weight"].as<double>();

            // SIMULATION
            N_horizon = config["simulation"]["N_horizon"].as<double>();
            sim_steps = config["simulation"]["sim_steps"].as<double>();
            step_per_phase = config["simulation"]["step_per_phase"].as<double>();
            c_target = config["simulation"]["c_target"].as<std::vector<double>>();
            
            // OBSTACLE
            obs_pos_init = config["obstacles"]["obs_pos_init"].as<std::vector<double>>();
            r_obs = config["obstacles"]["r_obs"].as<double>();
            speed = config["obstacles"]["speed"].as<double>();
            y_dot_max = config["obstacles"]["y_dot_max"].as<double>();
            top_pos_dyn = config["obstacles"]["top_pos_dyn"].as<std::vector<double>>();
            bot_pos_dyn = config["obstacles"]["bot_pos_dyn"].as<std::vector<double>>();
            obs_type = config["obstacles"]["obs_type"].as<std::string>();    
            obs_center = config["obstacles"]["obs_center"].as<std::vector<double>>();

            std::cout << "[Config] File YAML loaded with success!" << std::endl;
        }  
        catch (const YAML::Exception& e) {
            std::cerr << "[Errore Config] Impossible to load YAML file: " << e.what() << "\n";
            exit(EXIT_FAILURE); // Stop program if file miss
        }

    }
}