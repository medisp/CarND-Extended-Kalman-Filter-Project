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

    // TODO: YOUR CODE HERE

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	// ... your code here
    if (estimations.size() != ground_truth.size() ) {

        cout << " The input does not match for estimations versus ground truth values" << endl;
        return rmse;
    }

	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
        // ... your code here
		rmse +=  ( (estimations[i]-ground_truth[i])^2 ) / estimations.size() ;


	}
	//calculate the mean
	// ... your code here
    rmse = rmse/ground_truth.size();
	//calculate the squared root
	// ... your code here
    rmse = sqrt(rmse);
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

	//TODO: YOUR CODE HERE
    //Hj(0,0)
    float px2 = px*px;
    float py2 = py*py;

    float denom = (px2+py2);
    float sqdenom = sqrt(denom);
    float wierdenom = denom * sqdenom ;
    Hj(0,2) = 0;
    Hj(0,3) = 0;
    Hj(1,2) = 0;
    Hj(1,3) = 0;

    //flag for divides by 0

    bool dividebyzero = false;

    if (sqdenom < 0.0001 ) {
        dividebyzero = true;
        cout << "CalculateJacobian () - Error - Division by Zero is " <<dividebyzero << endl;
        return Hj;
    } else {
        Hj(0,0) = px/sqdenom ;
        Hj(0,1) = py/sqdenom ;
        Hj(1,0) = -py/denom ;
        Hj(1,1) = px/denom ;
        Hj(2,0) = py*(vx*py - vy*px) /wierdenom ;
        Hj(2,1) = px*(px*vy - py*vx) / wierdenom ;
        Hj(2,2) = px/sqdenom;
        Hj(2,3) = py/sqdenom;
    }
    return Hj;


}
