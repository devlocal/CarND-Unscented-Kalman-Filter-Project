#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include "nis.h"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  static constexpr int n_x_ = 5;

  ///* Augmented state dimension
  static constexpr int n_aug_ = 7;

  // Measurement dimension, radar can measure r, phi, and r_dot
  int n_z_radar_ = 3;
  int n_z_lidar_ = 2;

  ///* Sigma point spreading parameter
  double lambda_;

  // Previous measurement timestamp
  long prev_timestamp_;

  // Measurement covariance matrix S
  MatrixXd S_radar_;
  MatrixXd S_lidar_;

  // Mean predicted measurement
  VectorXd z_pred_radar_;
  VectorXd z_pred_lidar_;

  // Measurement noise covariance
  MatrixXd R_radar_;
  MatrixXd R_lidar_;

  // Matrix for sigma points in measurement space
  MatrixXd Zsig_radar_;
  MatrixXd Zsig_lidar_;

  ukf::Nis nis_radar_;
  ukf::Nis nis_lidar_;

  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(const MeasurementPackage &meas_package);

  /**
   * Generates sigma points
   */
  MatrixXd GenerateSigmaPoints();

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void PredictSigmaPoints(double delta_t, MatrixXd &Xsig_aug);

  /**
   * Predicts Mean and Covariance matrices
   */
  void PredictMeanAndCovariance();

  /**
   * Predicts radar measurement
   */
  void PredictRadarMeasurement();

  /**
   * Predicts lidar measurement
   */
  void PredictLidarMeasurement();

  /**
   * Normalizes angle
   * @param angle arbitrary angle value
   * @return angle value in range -pi..+pi
   */
  static double NormalizeAngle(double angle);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(const MeasurementPackage &meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(const MeasurementPackage &meas_package);
};

#endif /* UKF_H */
