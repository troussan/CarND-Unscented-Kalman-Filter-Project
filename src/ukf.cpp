#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <tkDecls.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // Radar measurements dimension
  n_z_ = 3;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

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

  noise_radar_ = MatrixXd(3, 3);
  noise_radar_  <<  std_radr_*std_radr_,  0.0,                      0.0,
                    0,                    std_radphi_*std_radphi_,  0.0,
                    0,                    0,                        std_radrd_*std_radrd_;
  // predicted sigma points matrix
  Xsig_pred_= MatrixXd(n_x_, 2 * n_aug_ + 1);

  // Sigma point spreading parameter
  lambda_= 3 - n_aug_;

  // Weights of sigma points
  weights_ = VectorXd(2*n_aug_ + 1);
  weights_.setConstant(0.5/(lambda_ + n_aug_));
  weights_[0] = lambda_/(lambda_ + n_aug_);

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates and initialize state.
      x_ <<
          meas_package.raw_measurements_[0] * cos(meas_package.raw_measurements_[1]),
          meas_package.raw_measurements_[0] * sin(meas_package.raw_measurements_[1]),
          0,
          0,
          0;

    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      //set the state with the initial location and zero velocity
      x_ <<
          meas_package.raw_measurements_[0],
          meas_package.raw_measurements_[1],
          0,
          0,
          0;

    }

    // done initializing, no need to predict or update
    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  //compute the time elapsed between the current and previous measurements
  double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in seconds
  time_us_ = meas_package.timestamp_;

//  double dt_2 = dt * dt;
//  double dt_3 = dt_2 * dt;
//  double dt_4 = dt_3 * dt;

//  //Modify the F matrix so that the time is integrated
//  ekf_.F_(0, 2) = dt;
//  ekf_.F_(1, 3) = dt;
//
//  //set the process covariance matrix Q
//  ekf_.Q_ = MatrixXd(4, 4);
//  ekf_.Q_ <<  dt_4/4*noise_ax_, 0,                dt_3/2*noise_ax_, 0,
//      0,                dt_4/4*noise_ay_, 0,                dt_3/2*noise_ay_,
//      dt_3/2*noise_ax_, 0,                dt_2*noise_ax_,   0,
//      0,                dt_3/2*noise_ay_, 0,                dt_2*noise_ay_;
  Prediction(dt);
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(meas_package);
  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    UpdateLidar(meas_package);
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
  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  GenerateAugmentedSigmaPoints(Xsig_aug);
  SigmaPointPrediction(Xsig_aug, delta_t, Xsig_pred_);
  PredictMeanAndCovariance();
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

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);


  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_, n_z_);

  PredictRadarMeasurement(Zsig, z_pred, S);
  UpdateState(meas_package, Zsig, z_pred, S);
}

void UKF::GenerateAugmentedSigmaPoints(MatrixXd &Xsig_out){

  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //create augmented mean state
  x_aug.head(n_x_) = x_;
  x_aug[n_x_] = 0.0;
  x_aug[n_x_ + 1] = 0.0;

  //create augmented covariance matrix
  P_aug.setZero();
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_ * std_a_;
  P_aug(n_x_ + 1 , n_x_+1) = std_yawdd_ * std_yawdd_;

  //create square root matrix
  MatrixXd A = P_aug.llt().matrixL();

  //create augmented sigma points
  double l_n_sqrt = sqrt(lambda_ + n_aug_);
  MatrixXd lnp = l_n_sqrt * A;
  Xsig_aug.col(0) = x_aug;
  for (int i = 1; i <= n_aug_; ++i) {
    Xsig_aug.col(i) =  x_aug + lnp.col(i-1);
    Xsig_aug.col(n_aug_ + i) =  x_aug - lnp.col(i-1);
  }

}

void UKF::SigmaPointPrediction(MatrixXd &Xsig_aug, double delta_t, MatrixXd &Xsig_pred) {

  int number_of_points = 2 * n_aug_ + 1;
  for(int i = 0; i < number_of_points; i++) {
    VectorXd x_k = Xsig_aug.col(i);
    double v = x_k(2);
    double phi = x_k(3);
    double phi_dot = x_k(4);
    double nu_a = x_k(5);
    double nu_phi = x_k(6);
    double delta_t_2o2 = delta_t*delta_t/2.0;
    double sin_phi = sin(phi);
    double cos_phi = cos(phi);
    VectorXd noise = VectorXd(n_x_);
    noise <<  delta_t_2o2 * cos_phi * nu_a,
              delta_t_2o2 * sin_phi * nu_a,
              delta_t * nu_a,
              delta_t_2o2 * nu_phi,
              delta_t * nu_phi;
    VectorXd change = VectorXd(n_x_);
    if (fabs(phi_dot) > 0.0000001) {
      change <<   (v/phi_dot) * (sin(phi + phi_dot*delta_t) - sin_phi),
                  (v/phi_dot) * (-cos(phi + phi_dot*delta_t) + cos_phi),
                  0.0,
                  phi_dot * delta_t,
                  0.0;
    } else {
      change <<   v* cos_phi * delta_t,
                  v* sin_phi * delta_t,
                  0.0,
                  0.0,
                  0.0;
    }
    Xsig_pred.col(i) = x_k.head(n_x_) + change + noise;
  }
}

void UKF::PredictMeanAndCovariance() {

  //create vector for weights
  VectorXd weights = VectorXd(2*n_aug_+1);

  //predict state mean
  x_ = Xsig_pred_ * weights;

  //predict state covariance matrix
  P_.setZero();
  for(int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd diff = Xsig_pred_.col(i) - x_;
    //normalize phi
    diff[3] -= trunc(diff[3]/TWO_PI) * TWO_PI;
    P_ += (weights[i] * (diff * diff.transpose()).array()).matrix();
  }
}

void UKF::PredictRadarMeasurement(MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S) {

  //transform sigma points into measurement space
  for (int i = 0; i < 2*n_aug_+1; i++) {
    double px = Xsig_pred_(0,i);
    double py = Xsig_pred_(1,i);
    double v = Xsig_pred_(2,i);
    double phi = Xsig_pred_(3,i);
    double px2py2 = px * px + py * py;
    if (abs(px) > 0.0001 && px2py2 > 0.0001) {
      double sqr = sqrt(px2py2);
      Zsig.col(i) << sqr, atan2(py, px), (px * cos(phi)*v + py * sin(phi) * v) / sqr;
    } else {
      cout << "Skipping update to avoid overflow/division by zero" << endl;
    }
  }

  //calculate mean predicted measurement
  z_pred = Zsig * weights_;

  //calculate measurement covariance matrix S
  S.setZero();
  for(int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd diff = Zsig.col(i) - z_pred;
    //normalize phi
    diff[1] -= trunc(diff[1]/TWO_PI) * TWO_PI;
    S += (weights_[i] * (diff * diff.transpose()).array()).matrix();
  }
  S += noise_radar_;
}

void UKF::UpdateState(MeasurementPackage meas_package, MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S) {

  //create example vector for incoming radar measurement
  VectorXd z = VectorXd(n_z_);
  z <<
    meas_package.raw_measurements_[0],   //rho in m
      meas_package.raw_measurements_[1],   //phi in rad
      meas_package.raw_measurements_[2];   //rho_dot in m/s

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_);

  //calculate cross correlation matrix
  Tc.setZero();
  for(int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //normalize phi
    x_diff[3] -= trunc(x_diff[3]/TWO_PI) * TWO_PI;
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //normalize phi
    z_diff[1] -= trunc(z_diff[1]/TWO_PI) * TWO_PI;
    Tc += (weights_[i] * (x_diff * z_diff.transpose()).array()).matrix();
  }

  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //update state mean and covariance matrix
  VectorXd z_diff = z - z_pred;
  z_diff[1] -= trunc(z_diff[1]/TWO_PI) * TWO_PI;

  x_ += K * (z_diff);

  P_ -= K * S * K.transpose();
}