#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */

	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0){
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //ToDO: YOUR CODE HERE 
  float t1 = px*px + py*py;

  //check division by zero
  if(fabs(t1) < 0.0001){
	cout << "CalculateJacobian () - Error - Division by Zero" << endl;
	t1 = 0.0001;
  }
  float t2 = px/sqrt(t1);
  float t3 = py/sqrt(t1);
  float t4 = t2*t2*t2;
    
  //compute the Jacobian matrix
  Hj << t2, t3, 0, 0,
        -py/t1, px/t1, 0, 0,
        py*(vx*py - vy*px)/(t4), px*(vy*px - vx*py)/t4, t2, t3;

	return Hj;
}
