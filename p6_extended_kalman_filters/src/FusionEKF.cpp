#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
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

  I_ = MatrixXd(4, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  ekf_.P_ = MatrixXd(4, 4);
  

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  ekf_.H_ = H_laser_;              

  I_ = MatrixXd::Identity(4, 4);
  
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
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      double rho, phi, px, py;

      rho = measurement_pack.raw_measurements_[0];
      phi = measurement_pack.raw_measurements_[1];

      px = cos(phi) * rho;
      py = sin(phi) * rho;

      ekf_.x_ << px, py, 0, 0;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      double px, py;

      px = measurement_pack.raw_measurements_[0];
      py = measurement_pack.raw_measurements_[1];

      ekf_.x_ << px, py, 0, 0;

    }

    ekf_.P_ <<  1000,  0,  0,    0,
                0,  1000,  0,    0,
                0,  0,  1000, 0,
                0,  0,  0,    1000;

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  double dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_ = MatrixXd(4, 4);

  ekf_.F_ <<  1,  0,  dt, 0,
              0,  1,  0,  dt,
              0,  0,  1,  0,
              0,  0,  0,  1;

  double dt4_4 = pow(dt, 4) / 4;
  double dt3_2 = pow(dt, 3) / 2;
  double dt2 =   pow(dt, 2);

  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt4_4 * 81, 0,          dt3_2 * 81, 0,
              0,          dt4_4 * 81, 0,          dt3_2 * 81,
              dt3_2 * 81, 0,          dt2 * 81,   0,
              0,          dt3_2 * 81, 0,          dt2 * 81;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    double px, py, vx, vy;

    px = ekf_.x_[0];
    py = ekf_.x_[1];
    vx = ekf_.x_[2];
    vy = ekf_.x_[3];

    double rho, phi, rhoDot;
    rho = sqrt(px * px + py * py);
    phi = atan2(py, px);
    rhoDot = (px * vx + py * vy) / sqrt(px * px + py * py);

    // predicted polar coordinates
    VectorXd xPolar(3);
    xPolar << rho, phi, rhoDot;

    VectorXd z(3);
    z <<  measurement_pack.raw_measurements_[0],
          measurement_pack.raw_measurements_[1],
          measurement_pack.raw_measurements_[2];

    VectorXd zDifference(3);
    zDifference = z - xPolar;

    // check phi in difference between measurement and predicted value
    if (zDifference[1] >= M_PI)
    {
      zDifference[1] -= 2*M_PI;
    }
    else if (zDifference[1] <= -M_PI)
    {
      zDifference[1] += 2*M_PI;
    }

    Tools tools;

    MatrixXd Hj(3,4);
    Hj = tools.CalculateJacobian(ekf_.x_);


    ekf_.R_ = R_radar_;    

    MatrixXd K;

    K = (ekf_.P_ * Hj.transpose()) *
        (Hj * ekf_.P_ * Hj.transpose() + ekf_.R_).inverse();

    ekf_.x_ = ekf_.x_ + K * (zDifference);


    ekf_.P_ = (I_ - K * Hj) * ekf_.P_;
  } else {
    // Laser updates
    ekf_.R_ = R_laser_;

    MatrixXd K;

    K =  (ekf_.P_ * ekf_.H_.transpose()) *
         (ekf_.H_ * ekf_.P_ * ekf_.H_.transpose() + ekf_.R_).inverse();

    double px = measurement_pack.raw_measurements_[0];
    double py = measurement_pack.raw_measurements_[1];

    VectorXd z(2);
    z << px, py;

    ekf_.x_ = ekf_.x_ + K * (z - ekf_.H_ * ekf_.x_);

    ekf_.P_ = (I_ - K * ekf_.H_) * ekf_.P_;
  }

  // print the output
  cout << "x_ =\n" << ekf_.x_ << endl << endl;
  cout << "P_ =\n" << ekf_.P_ << endl;
  cout << endl << endl;
}
