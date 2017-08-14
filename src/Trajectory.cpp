#include <iostream>
#include "Trajectory.h"
#include "Utils.hpp"
#include <assert.h>
#include <limits>
#include <assert.h>


using namespace std;

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Vector3d;


Trajectory::Trajectory()
	: timeStart_(0)
	, durationS_(0)
	, durationD_(0)
	, startState_(VectorXd::Zero(6))
	, endState_(VectorXd::Zero(6))
	, S_coeffs_(VectorXd::Zero(6))
	, D_coeffs_(VectorXd::Zero(6))
{
}

Trajectory::Trajectory(const VectorXd& startStateX6, const VectorXd& endStateX6, double durationS, double durationD, double timeStart)
	: timeStart_(timeStart)
	, durationS_(durationS)
	, durationD_(durationD)
	, startState_(startStateX6)
	, endState_(endStateX6)
	, S_coeffs_(VectorXd::Zero(6))
	, D_coeffs_(VectorXd::Zero(6))
{
	S_coeffs_ = CalcQuinticPolynomialCoeffs(startStateX6.head(3), endStateX6.head(3), durationS_);
	D_coeffs_ = CalcQuinticPolynomialCoeffs(startStateX6.segment(3,3), endStateX6.segment(3,3), durationD_);
}


TrajectoryPtr Trajectory::VelocityKeeping_STrajectory( const VectorXd& currStateX6,
													   double endD,
													   double targetVelocity,
													   double currTime,
													   double timeDurationS,
													   double timeDurationD)
{
	TrajectoryPtr pTraj = std::make_shared<Trajectory>();
	pTraj->timeStart_ = currTime;
	pTraj->durationS_ = timeDurationS;
	pTraj->durationD_ = timeDurationD;
	pTraj->startState_  = currStateX6;
	pTraj->endState_(3) = endD;

	const double currD = currStateX6(3);

	pTraj->S_coeffs_ = calc_s_polynomial_velocity_keeping(currStateX6.head(3), targetVelocity, timeDurationS);

	if (currD == endD)
	{
		pTraj->D_coeffs_(0) = currStateX6(3); // saves D position on the road.
	}
	else
	{
		pTraj->D_coeffs_ = CalcQuinticPolynomialCoeffs(pTraj->startState_.segment(3, 3), pTraj->endState_.segment(3, 3), timeDurationD);
	}

	pTraj->DE_V = targetVelocity;

	return pTraj;
}


// Used for prediction position other vehicles: constant velocity model without changing lane is used for small time delays.
/*TrajectoryPtr Trajectory::ConstartVelocity_STrajectory(double currS, double currD, double currVelosity, double currTime, double timeDuration)
{
	// Constant velosity trajectory with saving without shanging lane:
	//  Start state: [s, s_d, 0, d, 0, 0] 
	//  End state:   [s+s_d*T, s_d, 0, d, 0, 0]

	TrajectoryPtr pTraj = std::make_shared<Trajectory>();

	pTraj->startState_ << currS, currVelosity, 0, currD, 0, 0;
	pTraj->endState_ << currS + currVelosity * timeDuration, currVelosity, 0, currD, 0, 0;

	pTraj->S_coeffs_ << currS, currVelosity, 0, 0, 0, 0;
	pTraj->D_coeffs_ << currD, 0, 0, 0, 0, 0;

	pTraj->timeStart_ = currTime;
	pTraj->durationS_ = timeDuration;

	return pTraj;
}*/

/*
VectorXd Trajectory::EvaluateStateAt(double time) const
{
	double t = time - timeStart_;
	VectorXd state = evalaluateStateAt(S_coeffs_, D_coeffs_, std::min(t, durationS_));

	// model with acceleration = 0
	if (t > durationS_)
	{
		const double dt = t - durationS_;
		state(0) += state(1) * dt;
		state(3) += state(4) * dt;
	}

	return state;
}*/


VectorXd Trajectory::EvaluateStateAt(double time) const
{
	double t = time - timeStart_;
//	VectorXd state = evalaluateStateAt(S_coeffs_, D_coeffs_, std::min(t, durationS_));
	VectorXd stateS = calc_polynomial_at (S_coeffs_, std::min(t, durationS_));
	VectorXd stateD = calc_polynomial_at (D_coeffs_, std::min(t, durationD_));

	// model with acceleration = 0
	if (t > durationS_)
	{
		stateS(0) += stateS(1) * (t - durationS_);
	}

	if (t > durationD_)
	{
		stateD(0) += stateD(1) * (t - durationD_);
	}

	VectorXd state(6);
	state << stateS, stateD;
	return state;
}

/*
VectorXd Trajectory::evalaluateStateAt(const Eigen::VectorXd& s_coeffsV6, const Eigen::VectorXd& d_coeffsV6, double time)
{
	VectorXd state = VectorXd::Zero(6);
	state << calc_polynomial_at(s_coeffsV6, time), calc_polynomial_at(d_coeffsV6, time);
	return state;
}*/


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

	// Using constraints:  start [S_0, S_d, S_dd]
	//                       end [S_f_target_d, S_f_dd=0, T_f]
	//   and 
	//
	// s(t) = s0 + s0_d * t + 0.5 * s0_dd * t^2 + a3 * t^3 + a4 * t^4 + a5 * t^5
	// s_d(t) = s0_d + s0_dd * t + 3*a3 * t^2 + 4*a4 * t^3 + 5*a5 * t^4
	// s_dd(t) = s0_dd + 6*a3 * t^1 + 12*a4 * t^2 + 20*a5 * t^3
	

	// S_0_d + S_0_dd*T + 3*a3*T^2 + 4*a4*T^3 = S_target_d
	// S_0_dd + 6*a3*T + 12*a4*T^2 = 0

	//  | 3T^2  4T^3 |   | a3 |   | S_f_target_d - (S_0_d + S_0_dd * T) |
	//  |            | x |    | = |                                     |
	//  | 6T   12T^2 |   | a4 |   | -S_0_dd                             |


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
double Trajectory::CalcJerkCost(double timeDuration, double& maxJs, double& maxJd) const
{
	// See: https://d17h27t6h515a5.cloudfront.net/topher/2017/July/595fd482_werling-optimal-trajectory-generation-for-dynamic-street-scenarios-in-a-frenet-frame/werling-optimal-trajectory-generation-for-dynamic-street-scenarios-in-a-frenet-frame.pdf

	maxJs = Utils::MinDoubleVal;
	maxJd = Utils::MinDoubleVal;

	double Cs = 0;
	double Cd = 0;

	// Topic VIII. EXPERIMENTS: With the cost weights klat close to klon, the car always drives wellbehaved right behind the leading car(not shown),
	// so, for the sake of clearness, we used klat << klon.

	const double Ws = 1.;
	const double Wd = 1.;// 2.;

	const int points = static_cast<int>(timeDuration / cost_dt_);

	for (double t = 0; t < timeDuration; t += cost_dt_)
	{
		const double Js = calc_polynomial_jerk_at(S_coeffs_, std::min(t, durationS_));
		const double Jd = calc_polynomial_jerk_at(D_coeffs_, std::min(t, durationD_));

/*		std::cout
			<< " t: " << t
			<< " Js: " << Js
			<< " Jd: " << Jd
			<< std::endl;*/

		Cs += Js*Js;
		Cd += Jd*Jd;

		maxJs = max(Js, maxJs);
		maxJd = max(Jd, maxJd);
	}

	return (Ws * Cs + Wd * Cd) / points;
}


double Trajectory::CalcAccelCost(double timeDuration) const
{
	double cost = 0;
	const double points = timeDuration / cost_dt_;

	for (double t = 0; t < timeDuration; t += cost_dt_)
	{
		const double as = calc_polynomial_accel_at(S_coeffs_, std::min(t, durationS_));
		const double ad = calc_polynomial_accel_at(D_coeffs_, std::min(t, durationD_));
		cost += as*as + ad*ad;

/*		std::cout
		<< " t: " << t
		<< " As: " << as
		<< " Ad: " << ad
		<< " C: " << cost
			<< std::endl;
			*/
	}

	return cost / points;
}


std::pair<double, double> Trajectory::MinMaxVelocity_S() const
{
	std::pair<double, double> minMax(
		Utils::MaxDoubleVal,// std::numeric_limits<double>::max(),
		Utils::MinDoubleVal//std::numeric_limits<double>::min()
	);

	for (double t = 0; t < durationS_; t += dt_)
	{
		const double v = calc_polynomial_velocity_at(S_coeffs_, t);
		minMax.first = std::min(v, minMax.first);
		minMax.second = std::max(v, minMax.second);
	}

	return minMax;
}


double Trajectory::CalcVelocityCost(double targetVelocity, double timeDuration) const
{
//	PrintState();

	double Cost = 0;
	double t = 0;
	const int points = static_cast<int>(timeDuration / cost_dt_);

	for (int i = 0; i < points; i++)
	{
		const double vs = calc_polynomial_velocity_at(S_coeffs_, std::min(t, durationS_));
		const double vd = calc_polynomial_velocity_at(D_coeffs_, std::min(t, durationD_));
		double v = std::sqrt(vs*vs + vd*vd);

		if (v >= targetVelocity)
		{
			// fix for simulator - speed jumps up!
			v = 1e6; 
		}

		Cost += std::pow(targetVelocity - v, 2);
		t += cost_dt_;
/*
		std::cout
			<< " t: " << t
			<< " vs: " << vs
			<< " vd: " << vd
			<< " Cost: " << Cost
			<< std::endl;
			*/
	}

	return Cost / points;
}


// Calculates minimum distance to specified trajectory.
// Returns pair like { min-distance, time }.
/*std::pair<double, double> Trajectory::CalcMinDistanceToTrajectory(const TrajectoryPtr trajectory, double timeStep) const
{
	double minDist = MaxDoubleVal;
	double minDistTime = 0;

	const double timeEnd = timeStart_ + durationS_;

	for (double t = timeStart_; t < timeEnd; t += timeStep)
	{
		VectorXd s1 = EvaluateStateAt(t);
		VectorXd s2 = trajectory->EvaluateStateAt(t);
		const double dist = Utils::distance(s1(0), s1(3), s2(0), s2(3));
		if (dist < minDist)
		{
			minDist = dist;
			minDistTime = t;
		}
	}

	return std::pair<double, double>(minDist, minDistTime);
}*/


double Trajectory::CalcSaferyDistanceCost(const MatrixXd& s2, double timeDuration) const
{
	double Cost = 0;
	const double dt = cost_dt_;

	double t = timeStart_;
	const int points = static_cast<int>(timeDuration / dt);

	for (int i = 0; i < points; i++)
	{
		const VectorXd s1 = EvaluateStateAt(t);
		const double Sdist = Utils::distance(s1(0), s2(i, 0));
		const double Ddist = Utils::distance(s1(3), s2(i, 1));
		double velocity = s1(1);

		Cost += calcSafetyDistanceCost(Sdist, Ddist, velocity);

		t += dt;
	}

	return Cost / points;
}


static double CostFunction(double aDistance, double minDistance, double kSafetyDistance, double koeff = 2)
{
	const double a = -koeff;
	const double eMD = std::exp(a * minDistance);
	const double eSD = std::exp(a * kSafetyDistance);
	const double eD = std::exp(a * aDistance);
	return (0.95*eD + 0.05*eMD - eSD) / (eMD - eSD);
}

inline double CalcLongitudialDistanceCost(double dist, double velocity)
{
	const double longMinDist = 10;
	const double longSafetyDist = 1.2 * velocity;
	if (dist > longSafetyDist)
		return 0;
	return CostFunction(dist, longMinDist, longSafetyDist);
}

inline double CalcLateralDistanceCost(double dist)
{
	const double latMinDist = 2;
	const double latSafetyDist = 3.75;
	if (dist > dist)
		return 0;
	return CostFunction(dist, latMinDist, latSafetyDist);
}

inline double Trajectory::calcSafetyDistanceCost(double longitudinalDist, double lateralDist, double velocity) const
{
//	const double longMinDist = 10;
//	const double longSafetyDist = 1.2 * aVelocity;
//	double longitudialCost = CostFunction(aLongitudinalDistance, longMinDist, longSafetyDist);
	double longitudialCost = CalcLongitudialDistanceCost(longitudinalDist, velocity);

//	const double latMinDist = 2;
//	const double latSafetyDist = 3.75;
//	double lateralCost = CostFunction(aLateralDistance, latMinDist, latSafetyDist);
	double lateralCost = CalcLateralDistanceCost(lateralDist);

	if (lateralDist > 4)
	{
		// other car on the next lane.
		return 0;
	}

	if (longitudinalDist < 4.5)
	{
		if (lateralDist > 2)
		{
			return lateralCost;
		}
		else
		{
			// already crashed!
			return 1000;
		}
	}

	return longitudialCost;
}

/*
void Trajectory::PrintState() const
{
	std::cout << "StartT: " << timeStart_
		<< " ----------- " << std::endl
		<< " Dursation: " << durationS_
		<< " ----------- " << endl
		<< " StartState: " << startState_
		<< " ----------- " << endl
		<< " EndState: " << endState_
		<< " ----------- " << endl
		<< " S_coeff: " << S_coeffs_
		<< " ----------- " << endl
		<< " D_coeffs_: " << D_coeffs_
		<< endl;
}*/

void Trajectory::PrintInfo()
{
	const std::pair<double, double>& MinMaxV = MinMaxVelocity_S();

	std::cout //<< "T: " << timeStart_
		<< " startD: " << getStartD()
		<< " endD: " << getTargetD()
		<< " C: " << getTotalCost()
		<< " DC: " << getSafetyDistanceCost()
		<< " VC: " << getVelocityCost()
		<< " JC: " << getJerkCost()
		<< " v_min: " << MinMaxV.first
		<< " v_max: " << MinMaxV.second
		<< endl;

}
