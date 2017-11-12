#include "ukf.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

///* State dimension
constexpr int UKF::n_x_;

///* Augmented state dimension
constexpr int UKF::n_aug_;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.30;

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
  Complete the initialization.
  */

  is_initialized_ = false;

  Xsig_pred_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //create vector for weights
  weights_ = VectorXd(2 * n_aug_ + 1);

  S_radar_ = MatrixXd(n_z_radar_ ,n_z_radar_);
  S_lidar_ = MatrixXd(n_z_lidar_ ,n_z_lidar_);

  z_pred_radar_ = VectorXd(n_z_radar_);
  z_pred_lidar_ = VectorXd(n_z_lidar_);

  //measurement covariance matrix S
  S_radar_ = MatrixXd(n_z_radar_, n_z_radar_);
  S_lidar_ = MatrixXd(n_z_lidar_, n_z_lidar_);

  // Measurement noise covariance
  R_radar_ = MatrixXd(n_z_radar_, n_z_radar_);
  R_radar_ <<
    std_radr_ * std_radr_, 0, 0,
    0, std_radphi_ * std_radphi_, 0,
    0, 0, std_radrd_ * std_radrd_;

  R_lidar_ = MatrixXd(n_z_lidar_, n_z_lidar_);
  R_lidar_ <<
    std_laspx_ * std_laspx_, 0,
    0, std_laspy_ * std_laspy_;

  // Matrix for sigma points in measurement space
  Zsig_radar_ = MatrixXd(n_z_radar_, 2 * n_aug_ + 1);
  Zsig_lidar_ = MatrixXd(n_z_lidar_, 2 * n_aug_ + 1);

  lambda_ = 3 - n_x_;
}

UKF::~UKF() = default;

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(const MeasurementPackage &meas_package) {
  if (!is_initialized_) {
    is_initialized_ = true;

    prev_timestamp_ = meas_package.timestamp_;

    x_.setZero();
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      x_[0] = meas_package.raw_measurements_[0];
      x_[1] = meas_package.raw_measurements_[1];
    } else {
      // Convert polar coordinates to cartesian
      float range = meas_package.raw_measurements_[0];
      float bearing = meas_package.raw_measurements_[1];
      x_[0] = range * cos(bearing);
      x_[1] = range * sin(bearing);
    }

    P_.setIdentity();

    return;
  }

  // 1. Generate sigma points
  MatrixXd Xsig_aug = GenerateSigmaPoints();

  // 2. Predict sigma points
  double delta_t = (meas_package.timestamp_ - prev_timestamp_) / 1000000.0;  // expressed in seconds
  PredictSigmaPoints(delta_t, Xsig_aug);
  prev_timestamp_ = meas_package.timestamp_;

  // 3. Predict mean and covariance
  PredictMeanAndCovariance();

  if (meas_package.sensor_type_ == MeasurementPackage::SensorType::RADAR) {
    // 4. Predict measurement
    PredictRadarMeasurement();

    // 5. Update state
    UpdateRadar(meas_package);
  } else {
    // 4. Predict measurement
    PredictLidarMeasurement();

    UpdateLidar(meas_package);
  }
}

MatrixXd UKF::GenerateSigmaPoints() {
  // Create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  // Create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.setZero();

  // Create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.setZero();

  // Create augmented mean state
  for (int i = 0; i < n_x_; ++i) {
    x_aug[i] = x_[i];
  }
//  x_aug.head(n_x_) = x_;
  x_aug.tail(n_aug_ - n_x_).setZero();

  // Create augmented covariance matrix
  P_aug.block(0, 0, n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_ * std_a_;
  P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

  // Create square root matrix
  MatrixXd A_aug = P_aug.llt().matrixL();

  // Create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  Xsig_aug.block(0, 1, n_aug_, n_aug_) = x_aug.replicate(1, n_aug_) + sqrt(lambda_ + n_aug_) * A_aug;
  Xsig_aug.block(0, 1 + n_aug_, n_aug_, n_aug_) = x_aug.replicate(1, n_aug_) - sqrt(lambda_ + n_aug_) * A_aug;

  return Xsig_aug;
}

void UKF::PredictMeanAndCovariance() {
  // Set weights
  weights_[0] = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < 2 * n_aug_ + 1; ++i) {
    weights_[i] = 1 / (2 * (lambda_ + n_aug_));
  }

  // Predict state mean
  x_ = Xsig_pred_.cwiseProduct(weights_.transpose().replicate(n_x_, 1)).rowwise().sum();

  // Predict state covariance matrix
  P_.setZero();
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd m = Xsig_pred_.col(i) - x_;
    m[3] = NormalizeAngle(m[3]);

    P_ += weights_[i] * (m * m.transpose());
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::PredictSigmaPoints(double delta_t, MatrixXd &Xsig_aug) {
  /**
  Estimate the object's location. Modify the state vector, x_.
  Predict sigma points, the state, and the state covariance matrix.
  */

  Xsig_pred_ = Xsig_aug.topRows(n_x_);

  double half_delta_t_2 = 0.5 * delta_t * delta_t;

  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    double v = Xsig_aug(2, i);
    double psi = Xsig_aug(3, i);
    double psi_dot = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i);
    double nu_psi = Xsig_aug(6, i);

    double psi_dot_delta_t = psi_dot * delta_t;

    if (abs(psi_dot) > 1e-10) {
      Xsig_pred_(0, i) += (v / psi_dot) * (sin(psi + psi_dot_delta_t) - sin(psi)) + half_delta_t_2 * cos(psi) * nu_a;
      Xsig_pred_(1, i) += (v / psi_dot) * (-cos(psi + psi_dot_delta_t) + cos(psi)) + half_delta_t_2 * sin(psi) * nu_a;
    } else {
      Xsig_pred_(0, i) += v * delta_t * cos(psi) + half_delta_t_2 * cos(psi) * nu_a;
      Xsig_pred_(1, i) += v * delta_t * sin(psi) + half_delta_t_2 * sin(psi) * nu_a;
    }

    Xsig_pred_(2, i) += delta_t * nu_a;
    Xsig_pred_(3, i) += psi_dot_delta_t + half_delta_t_2 * nu_psi;
    Xsig_pred_(4, i) += delta_t * nu_psi;
  }
}

void UKF::PredictRadarMeasurement() {
  //transform sigma points into measurement space
  VectorXd px = Xsig_pred_.row(0);
  VectorXd py = Xsig_pred_.row(1);
  VectorXd v = Xsig_pred_.row(2);
  VectorXd phi = Xsig_pred_.row(3);

  Zsig_radar_.row(0) = (px.cwiseProduct(px) + py.cwiseProduct(py)).cwiseSqrt();

  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    if (abs(px[i]) > 1e-10) {
      Zsig_radar_(1, i) = atan2(py[i], px[i]);
    } else {
      Zsig_radar_(1, i) = 0;
    }
    double denominator = px[i] * px[i] + py[i] * py[i];
    if (abs(denominator) > 1e-10) {
      Zsig_radar_(2, i) = (px[i] * cos(phi[i]) * v[i] + py[i] * sin(phi[i]) * v[i]) / sqrt(denominator);
    } else {
      Zsig_radar_(2, i) = 0;
    }
  }

  //calculate mean predicted measurement
  z_pred_radar_ = Zsig_radar_.cwiseProduct(weights_.transpose().replicate(n_z_radar_, 1)).rowwise().sum();

  //calculate measurement covariance matrix S
  S_radar_.setZero();
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd m = Zsig_radar_.col(i) - z_pred_radar_;
    m[1] = NormalizeAngle(m[1]);

    S_radar_ += weights_[i] * (m * m.transpose());
  }

  // Add measurement noise covariance
  S_radar_ += R_radar_;
}

void UKF::PredictLidarMeasurement() {
  //transform sigma points into measurement space
  VectorXd px = Xsig_pred_.row(0);
  VectorXd py = Xsig_pred_.row(1);

  Zsig_lidar_.row(0) = px;
  Zsig_lidar_.row(1) = py;

  //calculate mean predicted measurement
  z_pred_lidar_ = Zsig_lidar_.cwiseProduct(weights_.transpose().replicate(n_z_lidar_, 1)).rowwise().sum();

  //calculate measurement covariance matrix S
  S_lidar_.setZero();
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd m = Zsig_lidar_.col(i) - z_pred_lidar_;
    m[1] = NormalizeAngle(m[1]);

    S_lidar_ += weights_[i] * (m * m.transpose());
  }

  // Add measurement noise covariance
  S_lidar_ += R_lidar_;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(const MeasurementPackage &meas_package) {
  /**
  Use lidar data to update the belief about the object's position.
  Modify the state vector, x_, and covariance, P_.
  Calculate the lidar NIS.
  */

  // Incoming lidar measurement
  const VectorXd &z = meas_package.raw_measurements_;

  nis_lidar_.Add(z_pred_lidar_, z, S_lidar_);

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_lidar_);

  //calculate cross correlation matrix
  Tc.setZero();
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    Tc += weights_[i] * ((Xsig_pred_.col(i) - x_) * (Zsig_lidar_.col(i) - z_pred_lidar_).transpose());
  }

  //calculate Kalman gain K
  MatrixXd K = Tc * S_lidar_.inverse();

  //update state mean and covariance matrix
  x_ += K * (z - z_pred_lidar_);

  P_ -= K * S_lidar_ * K.transpose();
}

double UKF::NormalizeAngle(double angle) {
  double a{angle};
  while (a > M_PI) {
    a = a - 2 * M_PI;
  }
  while (a < -M_PI) {
    a = a + 2 * M_PI;
  }
  return a;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(const MeasurementPackage &meas_package) {
  /**
  Use radar data to update the belief about the object's position.
  Modify the state vector, x_, and covariance, P_.
  Calculate the radar NIS.
  */

  // Incoming radar measurement
  const VectorXd &z = meas_package.raw_measurements_;

  nis_radar_.Add(z_pred_radar_, z, S_radar_);

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_radar_);

  //calculate cross correlation matrix
  Tc.setZero();
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    VectorXd dx = Xsig_pred_.col(i) - x_;
    VectorXd dz = Zsig_radar_.col(i) - z_pred_radar_;

    dx[3] = NormalizeAngle(dx[3]);
    dz[1] = NormalizeAngle(dz[1]);

    Tc += weights_[i] * dx * dz.transpose();
  }

  //calculate Kalman gain K
  MatrixXd K = Tc * S_radar_.inverse();

  VectorXd m = z - z_pred_radar_;
  m[1] = NormalizeAngle(m[1]);

  //update state mean and covariance matrix
  x_ += K * m;

  P_ -= K * S_radar_ * K.transpose();
}
