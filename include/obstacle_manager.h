#pragma once

#include <vector>
#include <array>
#include <string>
#include <memory>
#include <cmath>
#include <random>


// Dictionary to store info about obstacles
struct ObstacleState {
    std::array<double, 2> pos;
    double r_obs;
    double y_dot_max;
    std::string type;
};


// Abstract class
class BaseObstacle {
protected:
    std::array<double, 2> pos_;
    double r_obs_;
    double speed_;
    double y_dot_max_;
    std::string obs_type_;

public:
    BaseObstacle(const std::array<double, 2>& start_pos, double radius, double speed, double y_dot_max, const std::string& obs_type);
    
    virtual ~BaseObstacle() = default; // Mandatory virtual destructor for polymorfismo

    // Pure virtual method
    virtual void update(double dt, double t_global, const std::array<double, 2>& robot_pos) = 0;

    ObstacleState getState() const;
};


// SubClasses
class StaticObstacle : public BaseObstacle {
public:
    StaticObstacle(const std::array<double, 2>& start_pos, double radius, double y_dot_max);
    void update(double dt, double t_global, const std::array<double, 2>& robot_pos) override;
};

class DynamicObstacle : public BaseObstacle {
private:
    std::array<double, 2> p_top_;
    std::array<double, 2> p_bot_;
    std::array<double, 2> dir_u_;
    double dist_tot_;

public:
    DynamicObstacle(const std::array<double, 2>& top_pos, const std::array<double, 2>& bot_pos, double radius, double speed, double y_dot_max);
    void update(double dt, double t_global, const std::array<double, 2>& robot_pos) override;
};

class AdversarialObstacle : public BaseObstacle {
public:
    AdversarialObstacle(const std::array<double, 2>& start_pos, double radius, double speed, double y_dot_max);
    void update(double dt, double t_global, const std::array<double, 2>& robot_pos) override;
};

class CircularObstacle : public BaseObstacle {
private:
    std::array<double, 2> center_;
    double path_radius_;
    double theta_0_;
    double omega_;

public:
    CircularObstacle(const std::array<double, 2>& start_pos, const std::array<double, 2>& center, double radius_obs, double speed, double y_dot_max);
    void update(double dt, double t_global, const std::array<double, 2>& robot_pos) override;
};

class RandomObstacle : public BaseObstacle {
private:
    std::array<double, 4> bounds_; // {x_min, x_max, y_min, y_max}
    double heading_;
    std::mt19937 gen_; // Random generator
    std::uniform_real_distribution<double> dist_noise_;

public:
    RandomObstacle(const std::array<double, 2>& start_pos, double radius, double speed, const std::array<double, 4>& bounds, double y_dot_max);
    void update(double dt, double t_global, const std::array<double, 2>& robot_pos) override;
};


// OBSTACLE MANAGER
class ObstacleManager {
private:
    size_t n_obs_max_;
    // Vector of unique pointers to manage subclasses wothout memory leaks
    std::vector<std::unique_ptr<BaseObstacle>> obstacles_; 

public:
    ObstacleManager(size_t n_obs_max);

    // Riceve la ownership del puntatore unico
    void add_obstacle(std::unique_ptr<BaseObstacle> obs);
    
    void update_all(double dt, double t_global, const std::array<double, 2>& robot_pos);
    
    std::vector<ObstacleState> get_active_obstacles() const;
};