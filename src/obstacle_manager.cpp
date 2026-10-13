#include "obstacle_manager.h"
#include "config.h"
#include <cmath>


// Constructor
ObstacleManager::ObstacleManager(){
    // Resolve the string only one time in the constructor
    if (Config::obs_type == "static") type_ = ObsType::STATIC;
    else if(Config::obs_type == "dynamic") type_ = ObsType::DYNAMIC;
    else if(Config::obs_type == "adversarial") type_ = ObsType::ADVERSARIAL;
    else if(Config::obs_type == "circular") type_ = ObsType::CIRCULAR;
    else type_ = ObsType::STATIC; // Default

    // Initialize current position
    current_pos_ = {Config::obs_pos_init[0], Config::obs_pos_init[1]};
}


std::array<double, 2> ObstacleManager::update(double t_global, double dt_step, const std::array<double, 2>& robot_pos){
    if (type_ == ObsType::STATIC){
        return current_pos_;
    }
    else if(type_ == ObsType::DYNAMIC){
        std::array<double, 2> p_top = {Config::top_pos_dyn[0], Config::top_pos_dyn[1]};
            std::array<double, 2> p_bot = {Config::bot_pos_dyn[0], Config::bot_pos_dyn[1]};
            // Direction vector and distance btw two points
            double vec_x = p_bot[0] - p_top[0];
            double vec_y = p_bot[1] - p_top[1];
            double dist_tot = std::sqrt(vec_x * vec_x + vec_y * vec_y);
            std::array<double, 2> dir_u;

            if(dist_tot > 1e-5){
                double dir_x = vec_x / dist_tot;
                double dir_y = vec_y / dist_tot;

                // Total distance 
                double s_traveled = Config::speed * t_global;
                // Position in the round trip cycle (andata e ritorno)
                double s_cycle = std::fmod(s_traveled, 2*dist_tot);

                if(s_cycle <= dist_tot){
                    // ANDATA
                    current_pos_[0] = p_top[0] + dir_x * s_cycle;
                    current_pos_[1] = p_top[1] + dir_y * s_cycle;
                }
                else{
                    // RITORNO
                    current_pos_[0] = p_bot[0] - dir_x * (s_cycle - dist_tot);
                    current_pos_[1] = p_bot[1] - dir_y * (s_cycle - dist_tot);
                }
            }else{
                current_pos_ = p_top;
            }
    }
    else if(type_ == ObsType::ADVERSARIAL){
        double dir_x = robot_pos[0] - current_pos_[0];
        double dir_y = robot_pos[1] - current_pos_[1];
        double dist_to_robot = std::sqrt(dir_x * dir_x + dir_y * dir_y);

        if (dist_to_robot > 1e-3){
            dir_x /= dist_to_robot;
            dir_y /= dist_to_robot;
        }

        current_pos_[0] += dir_x * Config::speed * dt_step;
        current_pos_[1] += dir_y * Config::speed * dt_step;
    }
    else if(type_ == ObsType::CIRCULAR){
        double center_x = Config::obs_center[0];
        double center_y = Config::obs_center[1];
        double start_x = Config::obs_pos_init[0];
        double start_y = Config::obs_pos_init[1];

        double dx = start_x - center_x;
        double dy = start_y - center_y;
        double radius = std::sqrt(dx * dx + dy * dy);

        if(radius < 1e-5){
            current_pos_ = {start_x, start_y};
        }else{
            double theta_0 = std::atan2(dy, dx);
            double omega = Config::speed / radius;

            current_pos_[0] = center_x + radius * std::cos(omega * t_global + theta_0);
            current_pos_[1] = center_y + radius * std::sin(omega * t_global + theta_0);
        }
    }

    return current_pos_;
}