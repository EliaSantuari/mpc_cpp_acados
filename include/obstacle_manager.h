#pragma once
#include <array>
#include <string>

// Use enum to maximize performance
enum class ObsType{STATIC, DYNAMIC, ADVERSARIAL, CIRCULAR};


class ObstacleManager{
public: 
    ObstacleManager();

    // Function called at each step to update and get the position
    std::array<double, 2> update(double t_global, double dt_step, const std::array<double, 2>& robot_pos);

    // Return the las position computed
    std::array<double, 2> get_position() const { return current_pos_; }

private:
    ObsType type_;
    std::array<double, 2> current_pos_;

};



