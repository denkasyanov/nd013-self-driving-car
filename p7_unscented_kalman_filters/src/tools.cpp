#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */

  VectorXd rmse(4);

  rmse << 0, 0, 0, 0;

  if (estimations.size() != ground_truth.size())
  {
    std::cout << "CalculateRMSE() error - estimation and ground truth vectors sizes don't match";
    return rmse;
  }

  if (estimations.size() == 0)
  {
    std::cout << "CalculateRMSE() error - Estimations vector is empty";
    return rmse;
  }

  for (int i = 0; i < estimations.size(); ++i)
  {
    VectorXd residual = estimations[i] - ground_truth[i];

    residual = residual.array() * residual.array();
    rmse += residual;
  }

  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}