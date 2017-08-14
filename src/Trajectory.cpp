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


///////////////////////////////////////////////////////////////////////////////////////////////////
Trajectory::Trajectory()
	: m_timeStart(0)
	, m_durationS(0)
	, m_durationD(0)
	, m_tragetVelocity(0)
	, m_startState(VectorXd::Zero(6))
	, m_endState(VectorXd::Zero(6))
	, m_Scoeffs(VectorXd::Zero(6))
	, m_Dcoeffs(VectorXd::Zero(6))
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////
Trajectory::Trajectory(const VectorXd& startStateX6, const VectorXd& endStateX6, double durationS, double durationD, double timeStart)
	: m_timeStart(timeStart)
	, m_durationS(durationS)
	, m_durationD(durationD)
	, m_tragetVelocity(0)
	, m_startState(startStateX6)
	, m_endState(endStateX6)
	, m_Scoeffs(VectorXd::Zero(6))
	, m_Dcoeffs(VectorXd::Zero(6))
{
	m_Scoeffs = CalcQuinticPolynomialCoeffs(startStateX6.head(3), endStateX6.head(3), m_durationS);
	m_Dcoeffs = CalcQuinticPolynomialCoeffs(startStateX6.segment(3,3), endStateX6.segment(3,3), m_durationD);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
TrajectoryPtr Trajectory::VelocityKeeping_STrajectory( const VectorXd& currStateX6,
													   double endD,
													   double targetVelocity,
													   double currTime,
													   double timeDurationS,
													   double timeDurationD)
{
	TrajectoryPtr pTraj = std::make_shared<Trajectory>();
	pTraj->m_timeStart = currTime;
	pTraj->m_durationS = timeDurationS;
	pTraj->m_durationD = timeDurationD;
	pTraj->m_startState  = currStateX6;
	pTraj->m_endState(3) = endD;
	pTraj->m_tragetVelocity = targetVelocity;

	const double currD = currStateX6(3);

	pTraj->m_Scoeffs = calc_s_polynomial_velocity_keeping(currStateX6.head(3), targetVelocity, timeDurationS);

	if (currD == endD)
	{
		pTraj->m_Dcoeffs(0) = currStateX6(3); // saves D position on the road.
	}
	else
	{
		pTraj->m_Dcoeffs = CalcQuinticPolynomialCoeffs(pTraj->m_startState.segment(3, 3), pTraj->m_endState.segment(3, 3), timeDurationD);
	}

	return pTraj;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
VectorXd Trajectory::EvaluateStateAt(double time) const
{
	double t = time - m_timeStart;
	VectorXd stateS = calc_polynomial_at (m_Scoeffs, std::min(t, m_durationS));
	VectorXd stateD = calc_polynomial_at (m_Dcoeffs, std::min(t, m_durationD));

	// model with acceleration = 0
	if (t > m_durationS)
	{
		stateS(0) += stateS(1) * (t - m_durationS);
	}

	if (t > m_durationD)
	{
		stateD(0) += stateD(1) * (t - m_durationD);
	}

	VectorXd state(6);
	state << stateS, stateD;
	return state;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
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

///////////////////////////////////////////////////////////////////////////////////////////////////
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

///////////////////////////////////////////////////////////////////////////////////////////////////
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

///////////////////////////////////////////////////////////////////////////////////////////////////
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

	const int points = static_cast<int>(timeDuration / m_costDT);

	for (double t = 0; t < timeDuration; t += m_costDT)
	{
		const double Js = calc_polynomial_jerk_at(m_Scoeffs, std::min(t, m_durationS));
		const double Jd = calc_polynomial_jerk_at(m_Dcoeffs, std::min(t, m_durationD));

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

///////////////////////////////////////////////////////////////////////////////////////////////////
double Trajectory::CalcAccelCost(double timeDuration) const
{
	double cost = 0;
	const double points = timeDuration / m_costDT;

	for (double t = 0; t < timeDuration; t += m_costDT)
	{
		const double as = calc_polynomial_accel_at(m_Scoeffs, std::min(t, m_durationS));
		const double ad = calc_polynomial_accel_at(m_Dcoeffs, std::min(t, m_durationD));
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

///////////////////////////////////////////////////////////////////////////////////////////////////
std::pair<double, double> Trajectory::MinMaxVelocity_S() const
{
	std::pair<double, double> minMax(
		Utils::MaxDoubleVal,// std::numeric_limits<double>::max(),
		Utils::MinDoubleVal//std::numeric_limits<double>::min()
	);

	for (double t = 0; t < m_durationS; t += m_dt)
	{
		const double v = calc_polynomial_velocity_at(m_Scoeffs, t);
		minMax.first = std::min(v, minMax.first);
		minMax.second = std::max(v, minMax.second);
	}

	return minMax;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
double Trajectory::CalcVelocityCost(double targetVelocity, double timeDuration) const
{
//	PrintState();

	double cost = 0;
	double t = 0;
	const int points = static_cast<int>(timeDuration / m_costDT);

	for (int i = 0; i < points; i++)
	{
		const double vs = calc_polynomial_velocity_at(m_Scoeffs, std::min(t, m_durationS));
		const double vd = calc_polynomial_velocity_at(m_Dcoeffs, std::min(t, m_durationD));
		double v = std::sqrt(vs*vs + vd*vd);

		if (v >= targetVelocity)
		{
			// fix for simulator - speed jumps up!
			v = 1e6; 
		}

		cost += std::pow(targetVelocity - v, 2);
		t += m_costDT;
/*
		std::cout
			<< " t: " << t
			<< " vs: " << vs
			<< " vd: " << vd
			<< " Cost: " << Cost
			<< std::endl;
			*/
	}

	return cost / points;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
double Trajectory::CalcSaferyDistanceCost(const MatrixXd& s2, double timeDuration) const
{
	double cost = 0;
	double t = m_timeStart;
	const int points = static_cast<int>(timeDuration / m_costDT);

	for (int i = 0; i < points; i++)
	{
		const VectorXd s1 = EvaluateStateAt(t);
		const double Sdist = Utils::distance(s1(0), s2(i, 0));
		const double Ddist = Utils::distance(s1(3), s2(i, 1));
		double velocity = s1(1);

		cost += CalcSafetyDistanceCost(Sdist, Ddist, velocity);

		t += m_costDT;
	}

	return cost / points;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
static double CostFunction(double dist, double minDist, double safetyDist, double koeff = 2)
{
	const double a = -koeff;
	const double eMD = std::exp(a * minDist);
	const double eSD = std::exp(a * safetyDist);
	const double eD = std::exp(a * dist);
	return (0.95*eD + 0.05*eMD - eSD) / (eMD - eSD);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/*static double CostFunction(double distance, double minDistance, double safetyDist, double koeff = 2)
{
	const double a = 2;
	if (distance > minDistance)
		return 0;
	return 1. + 0.95 * (exp(-a*(distance - minDistance)) - 1);
}*/


///////////////////////////////////////////////////////////////////////////////////////////////////
inline double CalcLongitudialDistanceCost(double dist, double velocity)
{
	const double longMinDist = 10;
	const double longSafetyDist = 1.2 * velocity;
	if (dist > longSafetyDist)
		return 0;
	return CostFunction(dist, longMinDist, longSafetyDist);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
inline double CalcLateralDistanceCost(double dist)
{
	const double latMinDist = 2;
	const double latSafetyDist = 3.8;
	if (dist > latSafetyDist)
		return 0;
	return CostFunction(dist, latMinDist, latSafetyDist);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
inline double Trajectory::CalcSafetyDistanceCost(double longitudinalDist, double lateralDist, double velocity) const
{
	double longitudialCost = CalcLongitudialDistanceCost(longitudinalDist, velocity);
	double lateralCost = CalcLateralDistanceCost(lateralDist);

	if (lateralDist > 3.8) // 4
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

///////////////////////////////////////////////////////////////////////////////////////////////////
inline double CalcLaneOffsetCost(double d)
{
	const int nLane = Utils::DtoLaneNumber(d);
	if (nLane == -1)
		return 1e6;
	const double laneCenter = Utils::LaneNumberToD(nLane);
	const double distToCenter = Utils::distance(d, laneCenter);
	const double minDist = Utils::LaneWidth / 2.;
	const double safetyDist = 0.25;
	return CostFunction(distToCenter, minDist, safetyDist);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
double Trajectory::CalcLaneOffsetCost(double timeDuration) const
{
	double cost = 0;
	double t = m_timeStart;
	const int points = static_cast<int>(timeDuration / m_costDT);

	for (int i = 0; i < points; i++)
	{
		const double d = calc_polynomial_distance_at(m_Dcoeffs, std::min(t, m_durationD));
		cost += CalcLaneOffsetCost(d);
		t += m_costDT;
	}

	return cost / points;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void Trajectory::PrintInfo()
{
	const std::pair<double, double>& MinMaxV = MinMaxVelocity_S();

	std::cout //<< "T: " << m_timeStart
//		<< " startD: " << GetStartD()
		<< " endD: " << GetTargetD()
		<< " C: " << GetTotalCost()
		<< " JC: " << GetJerkCost()
		<< " VC: " << GetVelocityCost()
		<< " DC: " << GetSafetyDistanceCost()
		<< " LC: " << GetLaneOffsetCost()
//		<< " v_min: " << MinMaxV.first
//		<< " v_max: " << MinMaxV.second
		<< endl;

}
