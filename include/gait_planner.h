#pragma once
#include <vector>
#include <array>
#include <utility> 

// Use a enum instead od string to maximize performance
enum class Leg{FL, FR, RL, RR};
using GaitPair = std::pair<Leg, Leg>;

class GaitPlanner{
public:
    // Constructor
    GaitPlanner(double off_x, double off_y, int steps_per_phase = 1);

    // Return the sequence of the legs through the horizon
    std::vector<GaitPair> get_gait_horizon(int step_index, int N);

    // Return the position in X and Y of the new hips correspondent to gait_pair
    std::array<double, 4> compute_hip_positions(const std::array<double, 3>& c_state, const GaitPair& gait_pair);

    // Return the offset of the single leg
    std::array<double, 2> get_hip_offset(Leg leg) const {
        return hip_offsets_[static_cast<int>(leg)];
    }


private:
    int steps_per_phase_;
    double off_x_;
    double off_y_;

    // Array to memorize offsets of each leg (i.e. the index correspond to enum Leg)
    std::array<std::array<double, 2>, 4> hip_offsets_;

    // Array that contains the 2 phases of trot
    std::array<GaitPair, 2> trot_pairs_;
};