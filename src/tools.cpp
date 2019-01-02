#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  if (estimations.size()==0)
  {
    cout << "Not Valid input of estimations!!!" << endl;
    return rmse;
  }
  if (estimations.size() != ground_truth.size())
  {
    cout << "the size of the estimations and ground_truth not match!!" << endl;
    return rmse;
  }

  for (size_t i=0; i < estimations.size(); ++i)
  {
    VectorXd error = estimations[i] - ground_truth[i];
    error = error.array()*error.array();
    rmse += error;
  }

  rmse = rmse / estimations.size();

  rmse = rmse.array().sqrt();

  return rmse;  
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  
  MatrixXd Hj(3, 4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float ro = sqrt(px*px + py*py);
  float ro2 = px*px + py*py;
  float ro3 = ro*ro2;

  if (ro < 0.0001)
  {
      cout << "divide by zero!!!" << endl;
      return Hj;
  }    

  Hj << px/ro, py/ro, 0, 0,
        -py/ro2, px/ro2,0,0,
        py*(vx*py-vy*px)/ro3, px*(vy*px-vx*py)/ro3, px/ro, py/ro;  

  return Hj;
}
