#include "FusionEKF.h"
#include "Eigen/Dense"
#include "tools.h"
#include <iostream>
#include <cmath>
#include "constants.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF()
{
    is_initialized_ = false;

    previous_timestamp_ = 0;

    // initializing matrices
	// measurement covariance matrix - laser
    R_laser_ = MatrixXd(2, 2); 
    R_laser_ << 0.0225, 0,
				0, 0.0225;
	ekf_.R_lidar = R_laser_;
	
	// measurement covariance matrix - radar
	R_radar_ = MatrixXd(3, 3);
    R_radar_ << 0.09, 0, 0,
				0, 0.0009, 0,
				0, 0, 0.09;
	ekf_.R_radar = R_radar_;
	
	// state vector - gets updated to its first value during the first measurement
    ekf_.x_ = VectorXd(4);
	
    // state covariance matrix P
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
				0, 1, 0, 0,
				0, 0, 1000, 0,
				0, 0, 0, 1000;

    // measurement matrix for Lidar
	ekf_.H_ = MatrixXd(2,4);
    H_laser_ = MatrixXd(2, 4);
	H_laser_ << 1, 0, 0, 0,
				0, 1, 0, 0;
	ekf_.H_ = H_laser_;

	// measurement matrix for Radar
	ekf_.Hj_ = MatrixXd(3,4);
	Hj_ = MatrixXd(3, 4); // will be initialized during measurements	
	
    // the initial transition matrix F_
    ekf_.F_ = MatrixXd(4, 4); // set during first measurement and so on
	ekf_.F_ <<  1, 0, 0, 0,
				0, 1, 0 ,0,
				0, 0, 1, 0,
				0, 0, 0, 1;
				
	// process noise
	ekf_.Q_ = MatrixXd(4, 4); // set during first measurement and so on	
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF()
{
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage& measurement_pack)
{
    /**
     * Initialization
     */
    if(!is_initialized_) {

	// track timestamp across steps
	previous_timestamp_ = measurement_pack.timestamp_;
	
	// first measurement
//	cout << "First measurement, EKF: " << endl;
	if(measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
	    // Convert radar from polar to cartesian coordinates and initialize state.
		double theta = measurement_pack.raw_measurements_[1];
		while(theta > constants::PI || theta < -constants::PI) {
			if(theta > constants::PI) theta -= 2*constants::PI;
			else theta += 2*constants::PI;
		}
		ekf_.x_(0) = measurement_pack.raw_measurements_[0] * cos(theta);
		ekf_.x_(1) = measurement_pack.raw_measurements_[0] * sin(theta);
		ekf_.x_(2) = 0;
		ekf_.x_(3) = 0;
//		ekf_.x_(2) = measurement_pack.raw_measurements_[2] * cos(theta); // see how this makes a difference
//		ekf_.x_(3) = measurement_pack.raw_measurements_[2] * sin(theta);
	} else if(measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
	    // Initialize state
		ekf_.x_ <<  measurement_pack.raw_measurements_[0],
					measurement_pack.raw_measurements_[1],
					0,
					0;
	}

	// done initializing, no need to predict or update
	is_initialized_ = true;
	return;
    }

    /**
     * Prediction
	*/
	 
	double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; // convert to micro seconds
	previous_timestamp_ = measurement_pack.timestamp_;
    double dt_2 = dt * dt;
    double dt_3 = dt_2 * dt;
    double dt_4 = dt_3 * dt;

    // Modify the F matrix so that the time is integrated
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;
	
	double noise_ax = 9, noise_ay = 9;
    // set the process covariance matrix Q
	// Using noise_ax = 9 and noise_ay = 9 for Q matrix
    ekf_.Q_ << 	dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
				0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
				dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
				0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;
		
	// Predict
    ekf_.Predict();

    /**
     * Update
     */
    if(measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
		// Radar updates
		// recover state parameters
		Hj_ = tools.CalculateJacobian(ekf_.x_);
		// if Hj_ encountered a division / zero scenario, it will return all zero matrix
		// In this case, use previous Hj_ matrix as that is the best estimate we have as of now.
		// Also, relying on the assumption that first measurement will not result in such a situation.
		// If it does, assigning all zeros will mess up the calculation as well. So?....
		if(Hj_(0,0)!=0){
			ekf_.Hj_ = Hj_;
			ekf_.UpdateEKF(measurement_pack.raw_measurements_);
		} else {
			ekf_.UpdateEKF(measurement_pack.raw_measurements_);
		}
    } else {
		// Laser updates
		ekf_.Update(measurement_pack.raw_measurements_);
    }

    // print the output
//    cout << "x_ = " << ekf_.x_ << endl;
//    cout << "P_ = " << ekf_.P_ << endl;
}
