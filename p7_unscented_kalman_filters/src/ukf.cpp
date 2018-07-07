#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);
  x_.fill(0.0);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_.fill(0.0);

  /**
   * PARAMETERS FOR TUNING
   */
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  // I've chosen .8 by trial and error
  // Initially I've set std_a_ equal 3 as recommended. I've aslo set std_yawdd_ = 3 to see what'll happen.
  // Such high value of std_yawdd_ allow the filter to model moving object that can change yaw angle VERY fast.
  // However, I thought that such high yawdd is unlikely in real life because std_yawdd_ = 3 corresponds to
  // maximum change in yaw turn rate ~ 6 rad/s^2.
  std_yawdd_ = .8;
  /**
   * END OF PARAMETERS FOR TUNING
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = (3 - n_aug_);

  is_initialized_ = false;

  weights_ = VectorXd(2 * n_aug_ + 1);

  // initialize weights here because they don't change
  weights_(0) = lambda_ / (lambda_ + n_aug_);

  double weight = 0.5  / (lambda_ + n_aug_);
  for (int i = 1; i < 2 * n_aug_ + 1; ++i)
  {
    weights_(i) = weight;
  }

  // initialize Rr_ (radar noise) and Rl_ (lidar noise)
  Rr_ = MatrixXd(3, 3);
  Rr_ << std_radr_ * std_radr_,                         0,                       0,
                             0, std_radphi_ * std_radphi_,                       0,
                             0,                         0, std_radrd_ * std_radrd_;

  Rl_ = MatrixXd(2, 2);
  Rl_ << std_laspx_ * std_laspx_,                       0,
                               0, std_laspy_ * std_laspy_;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  if (!is_initialized_) {
    time_us_ = meas_package.timestamp_;
    if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      double px = meas_package.raw_measurements_[0];
      double py = meas_package.raw_measurements_[1];

      double v = 0;
      double yaw = 0;
      double yawd = 0;

      x_ << px, py, v, yaw, yawd;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      double rho = meas_package.raw_measurements_[0];
      double phi = meas_package.raw_measurements_[1];
      // double rhod = meas_package.raw_measurements_[2];

      double px = cos(phi) * rho;
      double py = sin(phi) * rho;

      double v = 0;
      double yaw = 0;
      double yawd = 0;

      x_ << px, py, v, yaw, yawd;
    }

    // we are relatively confident in px and py values
    P_ << 1, 0,  0,  0,  0,
          0, 1,  0,  0,  0,
          0, 0, 10,  0,  0,
          0, 0,  0, 10,  0,
          0, 0,  0,  0, 10;

    is_initialized_ = true;
    return;
  }

  // prediction step
  double delta_t = double(meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  while(delta_t > 0.1){
    const double delta_t_eps = 0.05;
    Prediction(delta_t_eps);
    delta_t -= delta_t_eps;
  }

  Prediction(delta_t);

  // update step
  if ((meas_package.sensor_type_ == MeasurementPackage::LASER) & (use_laser_))
  {
    UpdateLidar(meas_package);
  }
  else if ((meas_package.sensor_type_ == MeasurementPackage::RADAR) & (use_radar_))
  {
    UpdateRadar(meas_package);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  /**
   * augment mean and covariance
   */
  x_aug_ = VectorXd(7);
  x_aug_.fill(0.0);
  x_aug_.head(5) = x_;

  P_aug_ = MatrixXd(n_aug_, n_aug_);
  P_aug_.fill(0.0);
  P_aug_.topLeftCorner(n_x_,n_x_) = P_;

  P_aug_(5,5) = std_a_ * std_a_;
  P_aug_(6,6) = std_yawdd_ * std_yawdd_;

  /**
   * generate sigma points
   */
  Xsig_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_.fill(0.0);

  MatrixXd sqrtP = P_aug_.llt().matrixL();

  Xsig_.col(0) = x_aug_;
  for (int i = 0; i < n_aug_; ++i)
  {
    Xsig_.col(i + 1)          = x_aug_ + sqrt(lambda_ + n_aug_) * sqrtP.col(i);
    Xsig_.col(i + 1 + n_aug_) = x_aug_ - sqrt(lambda_ + n_aug_) * sqrtP.col(i);
  }

  /**
   * predict sigma points
   */
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  Xsig_pred_.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    // extract values for readability
    double px = Xsig_(0, i);
    double py = Xsig_(1, i);
    double v = Xsig_(2, i);
    double yaw = Xsig_(3, i);
    double yawd = Xsig_(4, i);
    double nu_a = Xsig_(5, i);
    double nu_yawdd = Xsig_(6, i);

    double px_p, py_p, v_p, yaw_p, yawd_p;
    if (fabs(yawd) > 0.001)
    {
      px_p = px + (v / yawd) * (sin(yaw + yawd * delta_t) - sin(yaw));
      py_p = py + (v / yawd) * (-cos(yaw + yawd * delta_t) + cos(yaw));
    }
    else
    {
      px_p = px + v * cos(yaw) * delta_t;
      py_p = px + v * sin(yaw) * delta_t;
    }

    // add noise
    px_p += 0.5 * delta_t * delta_t * cos(yaw) * nu_a;
    py_p += 0.5 * delta_t * delta_t * sin(yaw) * nu_a;

    v_p = v + delta_t * nu_a;
    yaw_p = yaw + yawd * delta_t + 0.5 * delta_t * delta_t * nu_yawdd;
    yawd_p = yawd + delta_t * nu_yawdd;

    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }

  /**
   * predict mean
   */
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    x_ += weights_(i) * Xsig_pred_.col(i);
  }

  /**
   * predict covariance
   */
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    while (x_diff(3) < -M_PI) x_diff(3) += 2 * M_PI;
    while (x_diff(3) > M_PI) x_diff(3) -= 2 * M_PI;

    P_ += weights_(i) * x_diff * x_diff.transpose();
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
// initialize vector that will contain predicted mean in measurement space

  MatrixXd Zsig_pred_ = MatrixXd(2, 2 * n_aug_ + 1);
  Zsig_pred_.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    double px = Xsig_pred_(0, i);
    double py = Xsig_pred_(1, i);

    Zsig_pred_(0, i) = px;
    Zsig_pred_(1, i) = py;
  }

  // calculate predicted mean in measurement space
  VectorXd z_pred_ = VectorXd(2);
  z_pred_.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    z_pred_ += weights_(i) * Zsig_pred_.col(i);
  }

  // calculate covariance matrix in measurement space
  MatrixXd S_ = MatrixXd(2,2);
  S_.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    VectorXd z_diff = Zsig_pred_.col(i) - z_pred_;
    S_ += weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise to covariance matrix
  S_ += Rl_;

  // calculate cross correlation matrix Tc_
  MatrixXd Tc_ = MatrixXd(n_x_, 2);
  Tc_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    VectorXd z_diff = Zsig_pred_.col(i) - z_pred_;
    Tc_ += weights_(i) * x_diff * z_diff.transpose();
  }
  VectorXd z = meas_package.raw_measurements_;

  MatrixXd K = Tc_ * S_.inverse();
  x_ = x_ + K * (z - z_pred_);
  P_ = P_ - K * S_ * K.transpose();
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  // initialize vector that will contain predicted mean in measurement space
  MatrixXd Zsig_pred_ = MatrixXd(3, 2 * n_aug_ + 1);
  Zsig_pred_.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    double px = Xsig_pred_(0, i);
    double py = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);
//    double yawd = Xsig_pred_(4, i);

    double rho = sqrt(px * px + py * py);
    double phi = atan2(py, px);
    double rhod = (px * cos(yaw) * v + py * sin(yaw) * v) / rho;

    Zsig_pred_(0, i) = rho;
    Zsig_pred_(1, i) = phi;
    Zsig_pred_(2, i) = rhod;
  }

  // calculate predicted mean in measurement space
  VectorXd z_pred_ = VectorXd(3);
  z_pred_.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    z_pred_ += weights_(i) * Zsig_pred_.col(i);
  }

  // calculate covariance matrix in measurement space
  MatrixXd S_ = MatrixXd(3,3);
  S_.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    VectorXd z_diff = Zsig_pred_.col(i) - z_pred_;
    while (z_diff(1) < -M_PI) z_diff(1) += 2 * M_PI;
    while (z_diff(1) > M_PI) z_diff(1) -= 2 * M_PI;

    S_ += weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise to covariance matrix
  S_ += Rr_;

  // calculate cross correlation matrix Tc_
  MatrixXd Tc_ = MatrixXd(n_x_, 3);
  Tc_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    while (x_diff(3) < -M_PI) x_diff(3) += 2 * M_PI;
    while (x_diff(3) > M_PI) x_diff(3) -= 2 * M_PI;

    VectorXd z_diff = Zsig_pred_.col(i) - z_pred_;
    while (z_diff(1) < -M_PI) z_diff(1) += 2 * M_PI;
    while (z_diff(1) > M_PI) z_diff(1) -= 2 * M_PI;

    Tc_ += weights_(i) * x_diff * z_diff.transpose();
  }
  VectorXd z = meas_package.raw_measurements_;

  MatrixXd K = Tc_ * S_.inverse();
  x_ = x_ + K * (z - z_pred_);
  P_ = P_ - K * S_ * K.transpose();
}
