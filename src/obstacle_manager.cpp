#include "obstacle_manager.h"
#include <iostream>
#include <algorithm>

const double PI = std::acos(-1.0);

// --- BaseObstacle ---
BaseObstacle::BaseObstacle(const std::array<double, 2>& start_pos, double radius, double speed, double y_dot_max, const std::string& obs_type)
    : pos_(start_pos), r_obs_(radius), speed_(speed), y_dot_max_(y_dot_max), obs_type_(obs_type) {}

ObstacleState BaseObstacle::getState() const {
    return {pos_, r_obs_, y_dot_max_, obs_type_};
}

// --- StaticObstacle ---
StaticObstacle::StaticObstacle(const std::array<double, 2>& start_pos, double radius, double y_dot_max)
    : BaseObstacle(start_pos, radius, 0.0, y_dot_max, "static") {}

void StaticObstacle::update(double dt, double t_global, const std::array<double, 2>& robot_pos) {
    // Stays still
}

// --- DynamicObstacle ---
DynamicObstacle::DynamicObstacle(const std::array<double, 2>& top_pos, const std::array<double, 2>& bot_pos, double radius, double speed, double y_dot_max)
    : BaseObstacle(top_pos, radius, speed, y_dot_max, "dynamic"), p_top_(top_pos), p_bot_(bot_pos) {
    
    double dx = p_bot_[0] - p_top_[0];
    double dy = p_bot_[1] - p_top_[1];
    dist_tot_ = std::sqrt(dx * dx + dy * dy);
    
    if (dist_tot_ > 1e-5) {
        dir_u_ = {dx / dist_tot_, dy / dist_tot_};
    } else {
        dir_u_ = {0.0, 0.0};
    }
}

void DynamicObstacle::update(double dt, double t_global, const std::array<double, 2>& robot_pos) {
    if (dist_tot_ > 1e-5) {
        double s_traveled = speed_ * t_global;
        double s_cycle = std::fmod(s_traveled, 2.0 * dist_tot_);
        
        if (s_cycle <= dist_tot_) {
            pos_[0] = p_top_[0] + dir_u_[0] * s_cycle;
            pos_[1] = p_top_[1] + dir_u_[1] * s_cycle;
        } else {
            pos_[0] = p_bot_[0] - dir_u_[0] * (s_cycle - dist_tot_);
            pos_[1] = p_bot_[1] - dir_u_[1] * (s_cycle - dist_tot_);
        }
    }
}

// --- AdversarialObstacle ---
AdversarialObstacle::AdversarialObstacle(const std::array<double, 2>& start_pos, double radius, double speed, double y_dot_max)
    : BaseObstacle(start_pos, radius, speed, y_dot_max, "adversarial") {}

void AdversarialObstacle::update(double dt, double t_global, const std::array<double, 2>& robot_pos) {
    double dir_x = robot_pos[0] - pos_[0];
    double dir_y = robot_pos[1] - pos_[1];
    double dist = std::sqrt(dir_x * dir_x + dir_y * dir_y);
    
    if (dist > 1e-3) {
        dir_x /= dist;
        dir_y /= dist;
        pos_[0] += dir_x * speed_ * dt;
        pos_[1] += dir_y * speed_ * dt;
    }
}

// --- CircularObstacle ---
CircularObstacle::CircularObstacle(const std::array<double, 2>& start_pos, const std::array<double, 2>& center, double radius_obs, double speed, double y_dot_max)
    : BaseObstacle(start_pos, radius_obs, speed, y_dot_max, "circular"), center_(center) {
    
    double dx = pos_[0] - center_[0];
    double dy = pos_[1] - center_[1];
    path_radius_ = std::sqrt(dx * dx + dy * dy);
    
    if (path_radius_ < 1e-5) {
        theta_0_ = 0.0;
        omega_ = 0.0;
    } else {
        theta_0_ = std::atan2(dy, dx);
        omega_ = speed_ / path_radius_;
    }
}

void CircularObstacle::update(double dt, double t_global, const std::array<double, 2>& robot_pos) {
    if (path_radius_ >= 1e-5) {
        pos_[0] = center_[0] + path_radius_ * std::cos(omega_ * t_global + theta_0_);
        pos_[1] = center_[1] + path_radius_ * std::sin(omega_ * t_global + theta_0_);
    }
}

// --- RandomObstacle ---
RandomObstacle::RandomObstacle(const std::array<double, 2>& start_pos, double radius, double speed, const std::array<double, 4>& bounds, double y_dot_max)
    : BaseObstacle(start_pos, radius, speed, y_dot_max, "random"), bounds_(bounds), dist_noise_(-1.0, 1.0) {
    
    std::random_device rd;
    gen_ = std::mt19937(rd());
    std::uniform_real_distribution<double> dist_heading(-PI, PI);
    heading_ = dist_heading(gen_);
}

void RandomObstacle::update(double dt, double t_global, const std::array<double, 2>& robot_pos) {
    double noise = dist_noise_(gen_) * dt;
    heading_ += noise;
    
    double vx = speed_ * std::cos(heading_);
    double vy = speed_ * std::sin(heading_);
    
    std::array<double, 2> next_pos = {pos_[0] + vx * dt, pos_[1] + vy * dt};
    
    double x_min = bounds_[0], x_max = bounds_[1];
    double y_min = bounds_[2], y_max = bounds_[3];
    
    if (next_pos[0] < x_min || next_pos[0] > x_max) {
        vx = -vx;
        heading_ = std::atan2(vy, vx);
        next_pos[0] = std::max(x_min, std::min(next_pos[0], x_max));
    }
    if (next_pos[1] < y_min || next_pos[1] > y_max) {
        vy = -vy;
        heading_ = std::atan2(vy, vx);
        next_pos[1] = std::max(y_min, std::min(next_pos[1], y_max));
    }
    
    pos_ = next_pos;
}

// --- ObstacleManager ---
ObstacleManager::ObstacleManager(size_t n_obs_max) : n_obs_max_(n_obs_max) {
    obstacles_.reserve(n_obs_max);
}

void ObstacleManager::add_obstacle(std::unique_ptr<BaseObstacle> obs) {
    if (obstacles_.size() < n_obs_max_) {
        // std::move cede la proprietà del puntatore intelligente al vettore
        obstacles_.push_back(std::move(obs)); 
    } else {
        std::cerr << "ERROR: Max obstacles reached (" << n_obs_max_ << ")\n";
    }
}

void ObstacleManager::update_all(double dt, double t_global, const std::array<double, 2>& robot_pos) {
    for (auto& obs : obstacles_) {
        obs->update(dt, t_global, robot_pos);
    }
}

std::vector<ObstacleState> ObstacleManager::get_active_obstacles() const {
    std::vector<ObstacleState> active;
    active.reserve(obstacles_.size());
    for (const auto& obs : obstacles_) {
        active.push_back(obs->getState());
    }
    return active;
}