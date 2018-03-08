# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

In this project, measurements from lidar and radar are used to estimate the position and velocities of a car along a trajectory. 
An Extended Kalman filter was used as a state estimator for this problem. The states are given by the car position and velocities in cartesian coordinates. 

The use of the Exteneded version of the filter comes from the fact that the radar measurements are in polar coordinates, making the measurement equation nonlinear. The computation of the Jacobian of the measurement equation is then necessary to linearize the equation, making the use of the classical Kalman filter possible. 

Measurements and process noises were provided. As a measure of filter performance, the Root Mean Squared Error was calculated. It would also be interesting to look at the characteristics of the innovation (mean, standard deviation). This is subject to future work. 

In this project, I:

* completed the RMSE, and the Jacobian for the measurement equation codes
* completed the `Predict()`,  `Update()` and `UpdateEKF` methods of the kalman_filter class, depending on the type of sensor data used
* initialized the initial state values, together with the measurement noise and the initial covariance matrices
* updated the state transition matrix `F` and the process noise covariance matrix `Q` according to the time elapsed since last measurement
* tested the implementation on the simulator data provided
* modified the initial state covariance and the process noise matrices to achieve better RMSE, by:
  * making the initial state covariance smaller, since the initial states are very close to the real ones; therefore there is no need for the filter to overshoot in the first time steps
  * making the process noise matrix slightly larger, to account for larger possible changes in velocity (turns with sharper radii, for example)

The RMSE error achieved when the filter was applied to Dataset1, with the alterations mentioned above, is: [0.0912, 0.0833, 0.3932, 0.4347] for the x position, y position, x velocity and y velocity respectively. 

The editor I used was Sublime 3.0.

