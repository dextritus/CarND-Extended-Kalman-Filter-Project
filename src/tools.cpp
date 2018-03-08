#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

  
  VectorXd rmse = VectorXd::Zero(4);
  //check for existing measurements and size requirements
  if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
  	cout << "Bad dimensions for estimations or ground truth" << endl;
  	return rmse;
  }
 

  int est_size = estimations.size();

  for (int i = 0; i<est_size; i++) {
  	VectorXd err = estimations[i] - ground_truth[i];
  	err = err.array() * err.array();
  	rmse += err;
  }

  rmse /= est_size;
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	MatrixXd Hj = MatrixXd::Zero(3, 4);

	// states
	double px = x_state(0);
	double py = x_state(1);
	double vx = x_state(2);
	double vy = x_state(3);

	//pre-compute terms
	double c1 = px*px+py*py;
	double c2 = sqrt(c1);
	double c3 = (c1*c2);

	//check for division by zero
	if(fabs(c1) < 0.0001){
		cout << "CalculateJacobian () - Error - Division by Zero, returning the same (null) jacobian" << endl;
		return Hj;
	}

	//compute the Jacobian matrix
	Hj << (px/c2), (py/c2), 0, 0,
		  -(py/c1), (px/c1), 0, 0,
		  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

	return Hj;
}
