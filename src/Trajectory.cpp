#include <iostream>
#include "Trajectory.h"
#include "Eigen-3.3/Eigen/Eigen"

using namespace std;

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Vector3d;


Trajectory::Trajectory()
	: timeStart_(0)
	, duration_(0)
	, cost_(0)
	, startState_(VectorXd::Zero(6))
	, endState_(VectorXd::Zero(6))
	, S_coeffs_(VectorXd::Zero(6))
	, D_coeffs_(VectorXd::Zero(6))
{
}

Trajectory::Trajectory(const VectorXd& startStateX6, const VectorXd& endStateX6, double duration, double timeStart)
	: timeStart_(timeStart)
	, duration_(duration)
	, cost_(0)
	, startState_(startStateX6)
	, endState_(endStateX6)
	, S_coeffs_(VectorXd::Zero(6))
	, D_coeffs_(VectorXd::Zero(6))
{
	S_coeffs_ = CalcQuinticPolynomialCoeffs(startStateX6.head(3), endStateX6.head(3), duration_);
	D_coeffs_ = CalcQuinticPolynomialCoeffs(startStateX6.segment(3,3), endStateX6.segment(3,3), duration_);
}


vector<VectorXd> Trajectory::getTrajectorySDPoints(double dt)
{
	vector<VectorXd> traj;
	VectorXd state = VectorXd::Zero(6);

	for (double t = 0; t < duration_; t += dt)
	{
		// calculate [s, s_d, s_dd] and [d, d_d, d_dd] at time 't'
		state << calc_polynomial_at(S_coeffs_, t), calc_polynomial_at(D_coeffs_, t);
		traj.push_back(state);
	}

	return traj;
}


VectorXd Trajectory::evalaluateStateAt(double time) const
{
//	assert(mIsFinalized);

	if (time > timeStart_ + duration_)
	{
		VectorXd state = evalaluateStateAt(S_coeffs_, D_coeffs_, duration_);
		VectorXd speedVector = VectorXd::Zero(6);
		speedVector(0) = state(1);
		speedVector(3) = state(4);
		state += (time - timeStart_ - duration_) * speedVector;
		return state;
	}
	else
	{
		return evalaluateStateAt(S_coeffs_, D_coeffs_, time - timeStart_);
	}
}


VectorXd Trajectory::evalaluateStateAt(const Eigen::VectorXd& s_coeffsV6, const Eigen::VectorXd& d_coeffsV6, double time)
{
	VectorXd state = VectorXd::Zero(6);
	state << calc_polynomial_at(s_coeffsV6, time), calc_polynomial_at(d_coeffsV6, time);
	return state;
}


VectorXd Trajectory::CalcQuinticPolynomialCoeffs(const VectorXd& startStateX3, const VectorXd& endStateX3, double T)
{
	const auto s_i = startStateX3[0];
	const auto s_i_d = startStateX3[1];
	const auto s_i_dd = startStateX3[2];

	const auto s_f = endStateX3[0];
	const auto s_f_d = endStateX3[1];
	const auto s_f_dd = endStateX3[2];

	const auto T2 = T * T;
	const auto T3 = T2 * T;
	const auto T4 = T3 * T;
	const auto T5 = T4 * T;

	MatrixXd A = MatrixXd(3, 3);
	A << T3,   T4,    T5,
		 3*T2, 4*T3,  5*T4,
		 6*T,  12*T2, 20*T3;

	VectorXd b = VectorXd(3);
	b << s_f - (s_i + s_i_d * T + 0.5 * s_i_dd * T2),
		 s_f_d - (s_i_d + s_i_dd * T),
	 	 s_f_dd - s_i_dd;

	Vector3d x = A.colPivHouseholderQr().solve(b);

	VectorXd coeffs = VectorXd(6);
	coeffs << s_i, s_i_d, 0.5*s_i_dd, x[0], x[1], x[2];

	return coeffs;
}


VectorXd Trajectory::calc_polynomial_at(const VectorXd& coeffsX6, double t)
{
	const VectorXd& a = coeffsX6;

	// s(t) = a0 + a1 * t + a2 * t^2 + a3 * t^3 + a4 * t^4 + a5 * t^5
	// s_d(t) = a1 + 2*a2 * t + 3*a3 * t^2 + 4*a4 * t^3 + 5*a5 * t^4
	// s_dd(t) = 2*a2 + 6*a3 * t^1 + 12*a4 * t^2 + 20*a5 * t^3

	// 3d Vector: [s, s_d, s_dd]
	VectorXd state = VectorXd::Zero(3);

	const auto t2 = t * t;
	const auto t3 = t2 * t;
	const auto t4 = t3 * t;
	const auto t5 = t4 * t;

	// calculate s(t)
	state(0) = a(0) + a(1) * t + a(2) * t2 + a(3) * t3 + a(4) * t4 + a(5) * t5;

	// calculate s_d(t)
	state(1) = a(1) + 2*a(2) * t + 3*a(3) * t2 + 4*a(4) * t3 + 5*a(5) * t4;

	// calculate s_dd(t)
	state(2) = 2*a(2) + 6*a(3) * t + 12*a(4) * t2 + 20*a(5) * t3;

//	std::cout << coeffsX6 << std::endl << std::endl;
//	std::cout << state << std::endl;

	return state;
}
