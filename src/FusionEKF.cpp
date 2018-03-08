#include "FusionEKF.h"
#include "Eigen/Dense"
#include "tools.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  // measurement covariance matrix - laser
  R_laser_ << 0.0225, 0, 
              0, 0.0225;

  // measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0, 
              0, 0.0009, 0, 
              0, 0, 0.09;

  //acceleration uncertainty (process noise)
  noise_ax = 20;
  noise_ay = 20;

  // initial state, in case no measurements
  ekf_.x_ = VectorXd(4);
  ekf_.x_ << 1, 1, 1, 1;

  //initial covariance matrix
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0, 
             0, 0, 10, 0, 
             0, 0, 0, 10;

  //allocate memory for the process noise covariance
  ekf_.Q_ = MatrixXd(4,4);

  // state transition
  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

  //lidar measurement matrix
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // first measurement
    cout << "EKF: " << endl;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      cout << "radar measurement init :"<<endl;
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      double rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float tan_phi = tan(phi);

      double pxx = rho * rho / (1 + tan_phi * tan_phi);
      double pyy = rho * rho - pxx;

      double px = sqrt(pxx);
      double py = sqrt(pyy);
      ekf_.x_ << px, py, 3, 0;
      cout <<" init x : "<<endl;
      cout << ekf_.x_ <<endl;
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      cout << "lidar measurement init : "<<endl;
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
      cout<< " init x : "<<endl;
      cout << ekf_.x_ <<endl;
    }

    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  //time step
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; 
  previous_timestamp_ = measurement_pack.timestamp_;

  double dtt = dt * dt;
  double dttt = dtt * dt;
  double dtttt = dtt * dtt;

  //update F
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
  
  //update Q
  ekf_.Q_ << dtttt/4*noise_ax, 0, dttt/2*noise_ax, 0,
             0, dtttt/4*noise_ay, 0, dttt/2*noise_ay,
             dttt/2*noise_ax, 0, dtt*noise_ax, 0,
             0, dttt/2*noise_ay, 0, dtt*noise_ay;

  //predict; linear equation, so no need for linearization
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    cout << " Radar measurement " << endl;
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = MatrixXd(3, 4);
    ekf_.H_ = Hj_;

    ekf_.R_ = MatrixXd(3, 3);
    ekf_.R_ = R_radar_;

    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
    cout << "Laser measurement "<<endl;
    ekf_.H_ = MatrixXd(2, 4);
    ekf_.H_ = H_laser_;

    ekf_.R_ = MatrixXd(2, 2);
    ekf_.R_ = R_laser_;

    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << endl;
  cout << "P_ = " << ekf_.P_ << endl;
  cout << endl;
}
