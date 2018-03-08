#include "kalman_filter.h"
#include <iostream>

using std::cout;
using std::endl;

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  MatrixXd Ft_ = F_.transpose();

  x_ = F_ * x_;
  P_ = F_ * P_ * Ft_ + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {

  VectorXd pred = H_ * x_;
  VectorXd y = z - pred;
  MatrixXd Ht_ = H_.transpose();
  MatrixXd S = H_ * P_ * Ht_ + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht_ * Si;

  // update state
  x_ = x_ + K * y;
  int x_size = x_.size();

  // update covariance
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  //use nonlinear mapping from cartesian to polar
  
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  double pxx = px*px;
  double pyy = py*py;

  double t1 = sqrt(pxx + pyy);

  VectorXd pred = VectorXd::Zero(3);
  
  float atanpypx = atan2(py,px);

  // fill in the predicted state
  pred(0) = t1;
  pred(1) = atanpypx;

  // check for division by zero when calculating phi
  if (fabs(t1)>0.00001) {
    pred(2) = (px * vx + py * vy) / t1;
  }

  VectorXd y = z - pred;
  //make sure phi error is between -PI and PI
  float y_phi = y(1);

  if (y_phi < -M_PI) {
    while (y_phi < -M_PI) 
      y_phi += 2 * M_PI;
  }
  else if (y_phi > M_PI) {
    while (y_phi > M_PI) 
      y_phi -= 2 * M_PI;
  } 
  //update phi error
  y(1) = y_phi;

  MatrixXd Ht_ = H_.transpose();
  MatrixXd S = H_ * P_ * Ht_ + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht_ * Si;

  // update state
  x_ = x_ + K * y;
  int x_size = x_.size();

  // update covariance
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
