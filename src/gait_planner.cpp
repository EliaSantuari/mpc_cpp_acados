#include "gait_planner.h"

#include <cmath>


GaitPlanner::GaitPlanner(double off_x, double off_y, int steps_per_phase) : steps_per_phase_(steps_per_phase), off_x_(off_x), off_y_(off_y){

    // Initialization of the offsets using enum order (FL=0, FR=1, RL=2, RR=3)
    hip_offsets_[static_cast<int>(Leg::FL)] = {off_x_, off_y_};
    hip_offsets_[static_cast<int>(Leg::FR)] = {off_x_, -off_y_};
    hip_offsets_[static_cast<int>(Leg::RL)] = {-off_x_, off_y_};
    hip_offsets_[static_cast<int>(Leg::RR)] = {-off_x_, -off_y_};

    // Definition of the two foot on ground
    trot_pairs_[0] = {Leg::FL, Leg::RR};
    trot_pairs_[1] = {Leg::FR, Leg::RL};
}


std::vector<GaitPair> GaitPlanner::get_gait_horizon(int step_index, int N){
    std::vector<GaitPair> horizon_pairs;
    horizon_pairs.reserve(N+1); // Pre-allocate the memory

    for (int k = 0; k <= N; ++k){
        int future_step = step_index + k;
        int future_phase = (future_step / steps_per_phase_) % 2;
        horizon_pairs.push_back(trot_pairs_[future_phase]);
    }

    return horizon_pairs;
}

std::array<double, 4> GaitPlanner::compute_hip_positions(const std::array<double, 3>& c_state, const GaitPair& gait_pair){

    double c_x = c_state[0];
    double c_y = c_state[1];
    double theta = c_state[2];

    // Pre-compute sin and cos
    double cos_t = std::cos(theta);
    double sin_t = std::sin(theta);

    // Take the offsets fo the specific leg
    const auto& off0 = hip_offsets_[static_cast<int>(gait_pair.first)];
    const auto& off1 = hip_offsets_[static_cast<int>(gait_pair.second)];

    // R @ offset + c_state
    double hip0_x = c_x + cos_t * off0[0] - sin_t * off0[1];
    double hip0_y = c_x + sin_t * off0[0] + cos_t * off0[1];

    double hip1_x = c_x + cos_t * off1[0] - sin_t * off1[1];
    double hip1_y = c_x + sin_t * off1[0] + cos_t * off1[1];

    return {hip0_x, hip0_y, hip1_x, hip1_y};
}