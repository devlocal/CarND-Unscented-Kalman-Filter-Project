#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
    * Calculate the RMSE here.
  */

  auto n_estimations = estimations.size();
  auto n_ground_truth = ground_truth.size();

  if (n_estimations == 0) {
    throw std::runtime_error("Cannot compute RMSE for an empty vector.");
  }

  if (n_estimations != n_ground_truth) {
    std::stringstream s;
    s << "Number of estimations ("
      << n_estimations << ") is not equal to the number of ground truth elements ("
      << n_ground_truth << "). Terminating.";
    throw std::runtime_error(s.str());
  }

  VectorXd rmse(estimations[0].size());
  rmse.setZero();

  for(int i = 0; i < n_estimations; ++i) {
    VectorXd d = estimations[i] - ground_truth[i];
    d = d.array() * d.array();
    rmse = rmse + d;
  }

  rmse = rmse / n_estimations;
  rmse = rmse.array().sqrt();

  return rmse;
}