#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

#define TWO_PI (2.0*M_PI)

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is true print NIS for lidar and radar
  bool nis_enabled_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

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
  double std_radrd_;

  ///* Radar noise measurements matrix
  MatrixXd noise_radar_;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Radar measurements dimension
  int n_z_radr;
  ///* Sigma point spreading parameter
  double lambda_;

  ///* Laser measurement matrix
  MatrixXd H_;

  ///* Laser measurement covariance matrix
  MatrixXd R_;

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
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);

private:
  int lidar_count_;
  int lidar_count_59_;
  int radar_count_;
  int radar_count_78_;

  void GenerateAugmentedSigmaPoints(MatrixXd &Xsig_out);

  void SigmaPointPrediction(MatrixXd &Xsig_aug, double delta_t);

  void PredictMeanAndCovariance();

  void PredictRadarMeasurement(MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S);

  void UpdateState(MeasurementPackage meas_package, MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S);

  void ClaculateNIS(VectorXd &z_diff, MatrixXd &S_inverted, int &count, int &count_78,
                    const char* sensorType, double cut_off);
};

#endif /* UKF_H */
