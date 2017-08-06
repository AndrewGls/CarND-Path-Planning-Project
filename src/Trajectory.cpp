#include <iostream>
#include "Trajectory.h"
#include <assert.h>
#include <limits>

using namespace std;

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Vector3d;


Trajectory::Trajectory()
	: startState_(VectorXd::Zero(6))
//	, endState_(VectorXd::Zero(6))
	, S_coeffs_(VectorXd::Zero(6))
	, D_coeffs_(VectorXd::Zero(6))
{
}

Trajectory::Trajectory(const VectorXd& startStateX6, const VectorXd& endStateX6, double duration, double timeStart)
	: startState_(startStateX6)
//	, endState_(endStateX6)
	, S_coeffs_(VectorXd::Zero(6))
	, D_coeffs_(VectorXd::Zero(6))
{
	S_coeffs_ = CalcQuinticPolynomialCoeffs(startStateX6.head(3), endStateX6.head(3), duration_);
	D_coeffs_ = CalcQuinticPolynomialCoeffs(startStateX6.segment(3,3), endStateX6.segment(3,3), duration_);
}


TrajectoryPtr Trajectory::VelocityKeeping_STrajectory(const VectorXd& currStateX6, double targetVelocity, double currTime, double timeDuration)
{
	TrajectoryPtr pTraj = std::make_shared<Trajectory>();
	pTraj->startState_ = currStateX6;
	pTraj->timeStart_ = currTime;
	pTraj->duration_ = timeDuration;

	pTraj->S_coeffs_ = calc_s_polynomial_velocity_keeping(currStateX6.head(3), targetVelocity, timeDuration);
	pTraj->D_coeffs_(0) = currStateX6(3); // saves D position on the road.

	pTraj->DE_V = targetVelocity;

	return pTraj;
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


void Trajectory::getTrajectorySDPointsTo(vector<Eigen::VectorXd>& outPoints, double dt)
{
	VectorXd state = VectorXd::Zero(6);
	size_t nIndex = 0;

	for (double t = 0; t < duration_; t += dt)
	{
		// calculate [s, s_d, s_dd] and [d, d_d, d_dd] at time 't'
		state << calc_polynomial_at(S_coeffs_, t), calc_polynomial_at(D_coeffs_, t);
		outPoints[nIndex] = state;
	}
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


Eigen::VectorXd Trajectory::calc_s_polynomial_velocity_keeping(const Eigen::VectorXd& startStateV3, double targetVelocity, double T)
{
	// Calculate 6 coeffs [a1, a2, a3, a4, a5] with constraints: a4=0 and a5 = 0.

	const double T2 = T * T;
	const double T3 = T2 * T;

	MatrixXd A(2, 2);
	A << 3*T2, 4*T3, 6*T, 12*T2;

	VectorXd b(2);
	b << targetVelocity - startStateV3(1) - startStateV3(2) * T, 0. - startStateV3(2);

	Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);
	//std::cout << "x: " << x << std::endl;
	assert(x(0) == x(0));
	assert(x(1) == x(1));

	VectorXd S_Coeffs(6);
	S_Coeffs << startStateV3(0), startStateV3(1), 0.5 * startStateV3(2), x, 0.;
	return S_Coeffs;
}


// Calculate Jerk cost function for evaluated trajectory points as array of vectors like [s, s_d, s_dd, d, d_d, d_dd].
double Trajectory::jerkCost_SD()
{
	double maxJs, maxJd;
	return jerkCost_SD (dt_, maxJs, maxJs);
}

// Calculate Jerk cost function for evaluated trajectory points as array of vectors like [s, s_d, s_dd, d, d_d, d_dd].
double Trajectory::jerkCost_SD(double timeStep, double& maxJs, double& maxJd)
{
	// See: https://d17h27t6h515a5.cloudfront.net/topher/2017/July/595fd482_werling-optimal-trajectory-generation-for-dynamic-street-scenarios-in-a-frenet-frame/werling-optimal-trajectory-generation-for-dynamic-street-scenarios-in-a-frenet-frame.pdf

	maxJs = -1e10;
	maxJd = -1e10;

	double Cs = 0;
	double Cd = 0;

	// Topic VIII. EXPERIMENTS: With the cost weights klat close to klon, the car always drives wellbehaved right behind the leading car(not shown),
	// so, for the sake of clearness, we used klat << klon.

	const double Ws = 1.;
	const double Wd = 2.;

	for (double t = 0; t < duration_; t += dt_)
	{
		const double Js = calc_polynomial_jerk_at(S_coeffs_, t);
		const double Jd = calc_polynomial_jerk_at(D_coeffs_, t);
		Cs += Js*Js;
		Cd += Jd*Jd;

		maxJs = max(Js, maxJs);
		maxJd = max(Jd, maxJd);
	}

	return Ws * Cs + Wd * Cd;
}

std::pair<double, double> Trajectory::MinMaxVelocity_S()
{
	return MinMaxVelocity_S(dt_);
}

std::pair<double, double> Trajectory::MinMaxVelocity_S (double timeStep)
{
	std::pair<double, double> minMax(
		1e10,// std::numeric_limits<double>::max(),
		-1e10//std::numeric_limits<double>::min()
	);

	for (double t = 0; t < duration_; t += dt_)
	{
		const double v = calc_polynomial_velocity_at(S_coeffs_, t);
		minMax.first = std::min(v, minMax.first);
		minMax.second = std::max(v, minMax.second);
	}

	return minMax;
}
