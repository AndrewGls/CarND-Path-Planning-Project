#include "KalmanFilter.h"


using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Vector3d;



KalmanFilter::KalmanFilter(double s, double d, double vs)
	: x_(VectorXd::Zero(4))
	, F_(MatrixXd::Identity(4, 4))
	, P_(MatrixXd::Identity(4, 4))
	, Q_(MatrixXd::Zero(4,4))
	, H_(MatrixXd::Zero(3, 4))
	, R_(MatrixXd::Identity(3, 3))
{
	x_ << s, d, vs, 0.;

	//P_.diagonal() << 1, 1, 1, 5;
	P_.diagonal() << 0.1, 0.1, 0.1, 10.;  // acceleration noise

	H_ << 1, 0, 0, 0,
		  0, 1, 0, 0,
		  0, 0, 1, 0;

	R_ *= 0.01; // the uncertainty in our sensor measurements.
}

VectorXd KalmanFilter::PredictStep(double dt)
{
	// Update F
	F_(0,2) = dt;
	F_(1,3) = dt;

	update_Q(dt);

	// predict state:
	x_ = F_ * x_;
	P_ = F_ * P_ * F_.transpose() + Q_;

	return x_;
}

void KalmanFilter::UpdateStep(double s, double d, double vs)
{
	const Vector3d z(s, d, vs); // 3x1
	const VectorXd y = z - H_ * x_; // 3x1
	const MatrixXd Ht = H_.transpose(); // 4x3
	const MatrixXd S = H_ * P_ * Ht + R_; // 3x3
	const MatrixXd K = P_ * Ht * S.inverse(); // 4x3

	// new state:
	x_ += (K * y);
	P_ -= K * H_ * P_;
}


void KalmanFilter::update_Q(double dt)
{
	const double dt_2 = dt * dt;
	const double dt_3 = dt_2 * dt;
	const double dt_4 = dt_3 * dt;
	constexpr double var_a = 9; // as suggested by the project.

	Q_ << dt_4/4*var_a,        0,        dt_3/2*var_a,      0,
		      0,         dt_4/4*var_a,        0,        dt_3/2*var_a,
	      dt_3/2*var_a,        0,        dt_2*var_a,        0,
		      0,         dt_3/2*var_a,        0,        dt_2*var_a;
}


//---------------------------------------------------------------------------------------
void KalmanFilter::SaveState()
{
	x_stack_.push(x_);
	P_stack_.push(P_);
}

void KalmanFilter::RestoreState()
{
	x_ = x_stack_.top();
	P_ = P_stack_.top();

	x_stack_.pop();
	P_stack_.pop();
}
