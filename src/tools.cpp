#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  //Validate parameters
  if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
    __throw_invalid_argument("Invalid estimation vector size");
  }

  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i){
    VectorXd diff = estimations[i] - ground_truth[i];
    rmse += (diff.array() * diff.array()).matrix();
  }

  rmse /= estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;

}