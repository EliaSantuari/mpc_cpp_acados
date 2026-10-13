#pragma once

#include <vector>
#include <array>
#include <string>


class DataLogger{
public: 
    // Constructor: pre-allocate memory for avoiding latency in real time
    DataLogger(int expected_steps, int n_horizon);

    // Store data of a single step
    void log_step(
        double comp_time,
        const std::array<double, 11>& x_curr,
        const std::array<double, 11>& u_opt,
        const std::array<double, 2>& obs_pos,
        const std::vector<std::array<double, 2>>& pred_op,
        const std::vector<std::array<double, 2>>& pred_pe
    );

    // At the end write on the file
    void save_to_csv(const std::string& filename) const;

private:
    int n_horizon_;

    std::vector<double> time_hist_;
    std::vector<std::array<double, 11>> history_X_;
    std::vector<std::array<double, 11>> history_U_;
    std::vector<std::array<double, 2>> history_obs_;  

    // For projection of opti-pessi save "flat" 2 dim array
    std::vector<std::vector<double>> history_pred_op_;
    std::vector<std::vector<double>> history_pred_pe_;
};