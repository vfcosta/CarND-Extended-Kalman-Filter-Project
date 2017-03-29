#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  // ... your code here
  if(estimations.size()==0 || estimations.size()!=ground_truth.size()) {
    cout << "Invalid input " << endl;
    return rmse;
  }

  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  //calculate the mean
  rmse /= estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  //check division by zero
  if(fabs(c1) < 0.0001){
    cout << "CalculateJacobian () - Error - Division by Zero. Returning an empty jacobian matrix" << endl;
    return Hj;
  }

  //compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
    -(py/c1), (px/c1), 0, 0,
    py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
}

VectorXd Tools::ConvertPolarToCartesian(const Eigen::VectorXd& measurements) {
  float p = measurements[0];
  float phi = measurements[1];
  float v = measurements[2];
  float cos_phi = cos(phi);
  float sin_phi = sin(phi);
  VectorXd cartesian = VectorXd(4);
  cartesian << p * cos_phi, p * sin_phi, v * cos_phi, v * sin_phi;
  return cartesian;
}

VectorXd Tools::ConvertCartesianToPolar(const Eigen::VectorXd& x) {
  VectorXd polar = VectorXd(3);
  float c1 = sqrt(pow(x[0], 2) + pow(x[1], 2));
  if (c1 > 0) {
    polar << c1, atan2(x[1], x[0]), (x[0]*x[2] + x[1]*x[3])/c1;
  }
  return polar;
}