#include "kalman_filter.h"
#include "math.h"
#include "constants.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Constructor
KalmanFilter::KalmanFilter() {}

// Destructor
KalmanFilter::~KalmanFilter() {}

// Predict step
void KalmanFilter::Predict() {
	
   x_ = F_ * x_;
   P_ = F_ * P_ * F_.transpose() + Q_;
   
   	// print the output
//   std::cout << "Predicted : x_ = " << x_ << std::endl;
//   std::cout << "Predicted : P_ = " << P_ << std::endl;
}

// Lidar/KF Update step
void KalmanFilter::Update(const VectorXd &z) {
	
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_lidar;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

// Convert vector from state space to radar measurement space
VectorXd KalmanFilter::MapToRadarSpace(const VectorXd& x) {
	VectorXd radar_measurement_ = VectorXd(3);
	
	radar_measurement_(0) = sqrt(pow(x(0),2) + pow(x(1),2));
	radar_measurement_(1) = atan2(x(1), x(0));
	if(fabs(radar_measurement_(0)) < 0.0001) {
		radar_measurement_(2) = 0;
	} else {
		radar_measurement_(2) = (x(0)*x(2) + x(1)*x(3)) / radar_measurement_(0);
	}
	
	return radar_measurement_;
}

// Radar/EKF Update
void KalmanFilter::UpdateEKF(const VectorXd &z) {
	// update the state by using Extended Kalman Filter equations

   	VectorXd z_pred = MapToRadarSpace(x_);
	VectorXd y = z - z_pred;
	// Make sure theta is in the range -pi to pi
	while(y(1) > constants::PI or y(1) < -constants::PI) {
		if(y(1) > constants::PI){
			y(1) -= 2*constants::PI;
		} else {
			y(1) += 2*constants::PI;
		}
	}
	MatrixXd Ht = Hj_.transpose();
	MatrixXd S = Hj_ * P_ * Ht + R_radar;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * Hj_) * P_;
}
