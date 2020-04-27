#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
 
Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
								  
	if(estimations.size() != ground_truth.size()) {
		std::cerr << "Size of estimations and ground truth arrays do not match." << std::endl;
		std::exit(1);
	}
	
	VectorXd rmse = VectorXd(estimations[0].size());
	rmse << 0, 0, 0, 0; // Initialize to zero, else if random values are assigned , calculation gets messed  up
	
	for(size_t i = 0; i < estimations.size(); ++i) {
		VectorXd error = estimations[i] - ground_truth[i];
		error = error.array().square();
		rmse += error;
	}

	rmse = rmse / estimations.size();
	rmse = rmse.array().sqrt();
	
//	std::cout << "RMSE : " << rmse(0) << " " << rmse(1) << " " << rmse(2) << " " << rmse(3) << std::endl;
	
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  // Calculate a Jacobian
	MatrixXd Hj_ = MatrixXd(3,4);
   
	double px = x_state(0);
	double py = x_state(1);
	double vx = x_state(2);
	double vy = x_state(3);

	// pre-compute a set of terms to avoid repeated calculation
	double c1 = px * px + py * py;
	double c2 = sqrt(c1);
	double c3 = (c1 * c2);

	// check division by zero
	if(fabs(c1) < 0.0001) {
		std::cerr << "CalculateJacobian () - Error - Division by Zero" << std::endl;
		Hj_ << 0, 0, 0, 0,
				0, 0, 0, 0,
				0, 0 ,0, 0;
		return Hj_;
	}

	// compute the Jacobian matrix
	Hj_ << (px / c2), (py / c2), 0, 0,
		  -(py / c1), (px / c1), 0, 0,
		  py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;

	return Hj_;
}
