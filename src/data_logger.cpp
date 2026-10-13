#include "data_logger.h"
#include <fstream>
#include <iostream>

// Constructor
DataLogger::DataLogger(int expected_steps, int n_horizon) : n_horizon_(n_horizon){

    // Pre-allocate memory
    time_hist_.reserve(expected_steps);
    history_X_.reserve(expected_steps);
    history_U_.reserve(expected_steps);
    history_obs_.reserve(expected_steps);
    history_pred_op_.reserve(expected_steps);
    history_pred_pe_.reserve(expected_steps);
}


void DataLogger::log_step(
    double comp_time,
    const std::array<double, 11>& x_curr,
    const std::array<double, 11>& u_opt,
    const std::array<double, 2>& obs_pos,
    const std::vector<std::array<double, 2>>& pred_op,
    const std::vector<std::array<double, 2>>& pred_pe)
{
    time_hist_.push_back(comp_time);
    history_X_.push_back(x_curr);
    history_U_.push_back(u_opt);
    history_obs_.push_back(obs_pos);

    // Flatten the predictions from array of arrays to single vector
    std::vector<double> flat_pred_op(n_horizon_ * 2);
    std::vector<double> flat_pred_pe(n_horizon_ * 2);

    for(int k = 0; k < n_horizon_; ++k){
        flat_pred_op[k*2] = pred_op[k][0];
        flat_pred_op[k*2 + 1] = pred_op[k][1];

        flat_pred_pe[k*2] = pred_pe[k][0];
        flat_pred_pe[k*2 + 1] = pred_pe[k][1];
    }

    history_pred_op_.push_back(flat_pred_op);
    history_pred_pe_.push_back(flat_pred_pe);

}


void DataLogger::save_to_csv(const std::string& filename) const {
    std::ofstream file(filename);

    if(!file.is_open()){
        std::cerr << "Error in opening the file " << filename << " for the saving." << std::endl;
        return;
    }

    // Header
    file << "comp_time,";
    // X (11)
    for(int i = 0; i<11; ++i) file << "x" << i << ",";
    // U (11)
    for(int i = 0; i<11; ++i) file << "u" << i << ",";
    // Obs (2)
    file << "obs_x,obs_y,";
    // Pred OP (N*2)
    for(int k = 0; k < n_horizon_; ++k) file << "pred_op_" << k << "_x,pred_op_" << k << "_y,";
    // Pred PE (N*2)
    for(int k = 0; k < n_horizon_; ++k){
        file << "pred_pe_" << k << "_x,pred_pe_" << k << "_y";
        if (k < n_horizon_ - 1) file << ",";
    }
    file << "\n";


    // Write data
    for (size_t i = 0; i < time_hist_.size(); ++i)
    {
        file << time_hist_[i] << ",";
        for (double val : history_X_[i]) file << val << ",";
        for (double val : history_U_[i]) file << val << ",";
        file << history_obs_[i][0] << "," << history_obs_[i][1] << ",";

        for (double val :history_pred_op_[i]) file << val << ",";
        for (size_t j = 0; j < history_pred_pe_[i].size(); ++j){
            file << history_pred_pe_[i][j];
            if (j < history_pred_pe_[i].size() - 1) file << ",";
        }
        file << "\n";
    }

    file.close();
    std::cout << "Simulation data saved with success in: " << filename << std::endl;
    
}