#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

    // calculating error for KF function
    VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
    // calculating Kalman matrix
	UpdateCommonStep(y);

}
void NormalizeAngle( double& phi) {

    phi = atan2(sin(phi),cos(phi));
}


void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
    /* transform measurements from to do measurement update from polar coordinates
     Radar measurements need to undergo h(x) function

    px is x_(0) || py is x_(1)
    vx is x_(2) || vy is x_(3)


    rho = sqrt(x^2+y^2)
    phi = arctan(y/x)
    drdt=(x*v1_y*vy)/rho ^^
    */
    double rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
    double phi = atan2(x_(1) / x_(0));
    double drdt = (x_(0)*x_(2) + x_(1)*x_(3)) / rho;
    VectorXd h = VectorXd(3);
    h << rho, phi, drdt;
	
    
    VectorXd y = z - h;

    // normalizing angle of y[1]
    NormalizeAngle(y[1]);
			    
// calculating Kalman matrix
	UpdateCommonStep(y);


}

KalmanFilter::UpdateCommonStep(const VectorXd& y) {

	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}


