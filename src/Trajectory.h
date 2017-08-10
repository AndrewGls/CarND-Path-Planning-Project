#pragma once

#include "Types.h"
#include <vector>
#include <memory>
#include <utility>
#include <cmath>


class Trajectory;
using TrajectoryPtr = std::shared_ptr<Trajectory>;


class Trajectory
{
public:
	Trajectory();

	// Input:
	//   startStateX6 is a vector of size 6: [s, s_d, s_dd, d, d_d, d_dd]
	//   endStateX6 is a vector of size 6:   [s, s_d, s_dd, d, d_d, d_dd]
	Trajectory(const Eigen::VectorXd& startStateX6, const Eigen::VectorXd& endStateX6, double duration, double timeStart);

	// Calculates Jerk Minimizing Trajectory for the Velocity Leeping state, using currect state, target velocity and duration T.
	// currStateX6 is 6-dim vector: [s, s_d, s_dd, d, d_d, d_dd].
	// Usually only the first 3 coord. [s, s_d, s_dd] is used for trajectory calculation (it is better to split to S-trajectory and D-trajectory).
	// https://d17h27t6h515a5.cloudfront.net/topher/2017/July/595fd482_werling-optimal-trajectory-generation-for-dynamic-street-scenarios-in-a-frenet-frame/werling-optimal-trajectory-generation-for-dynamic-street-scenarios-in-a-frenet-frame.pdf
	static TrajectoryPtr VelocityKeeping_STrajectory (const Eigen::VectorXd& currStateX6, double endD, double targetVelocity, double currTime, double timeDuration);

	// Used for prediction position other vehicles: constant velocity model without changing lane is used for small time delays.
	static TrajectoryPtr ConstartVelocity_STrajectory(double currS, double currD, double currVelosity, double currTime, double timeDuration);

	// Returns vector state [s, s_d, s_dd, d, d_d, d_dd] at the time 'time'.
	Eigen::VectorXd EvaluateStateAt(double time) const;

	// Calculates minimum distance to specified trajectory.
	// Returns pair like { min-distance, time }.
	std::pair<double, double> CalcMinDistanceToTrajectory(const TrajectoryPtr otheTraj, double timeStep = delta_t) const;
	
	double calcSaferyDistanceCost(const Eigen::MatrixXd& s2, double timeDuration) const;

	// Returns trajectory duration in seconds.
	double getDuration() const { return duration_; }

	Eigen::VectorXd getStartState() const { return startState_; }


	double getTotalCost() const { return cost_.sum(); }

	void setTimeCost (double cost)			{ cost_(0) = cost; }
	void setJerkCost (double cost)			{ cost_(1) = cost; }
	void setVelocityCost (double cost)		{ cost_(2) = cost; }
	void setSafetyDistanceCost (double cost)	{ cost_(3) = cost; }

	double getTimeCost() const			{ return cost_(0); }
	double getJerkCost() const			{ return cost_(1); }
	double getVelocityCost() const		{ return cost_(2); }
	double getSafetyDistanceCost() const { return cost_(3); }


	// Calculation of cost functions.
	// See: https://d17h27t6h515a5.cloudfront.net/topher/2017/July/595fd482_werling-optimal-trajectory-generation-for-dynamic-street-scenarios-in-a-frenet-frame/werling-optimal-trajectory-generation-for-dynamic-street-scenarios-in-a-frenet-frame.pdf

	// Calculate Jerk cost function for evaluated trajectory points as array of vectors like [s, s_d, s_dd, d, d_d, d_dd].
	double jerkCost_SD(double timeDuration, double& maxJs, double& maxJd);
	double velocityCost_S(double targetVelocity, double timeDuration) const;
	std::pair<double, double> MinMaxVelocity_S();

	void PrintInfo();

private:
	// Calculates Jerk Minimizing Trajectory for start state, end state and duration T.
	// Input:
	//   startStateX3 is vector of size 3: [s, s_d, s_dd]
	//   endStateX3 is vector of size 3:   [s, s_d, s_dd]
	// Returns:
	//   vector of size 6: [s, s_d, s_dd, a3, a4, a5]
	static Eigen::VectorXd CalcQuinticPolynomialCoeffs(const Eigen::VectorXd& startStateX3, const Eigen::VectorXd& endStateX3, double T);

	// Calculates polynomial value at time 't' and returns 6-dim vector: [s, s_d, s_dd, d, d_d, d_dd].
	static Eigen::VectorXd evalaluateStateAt(const Eigen::VectorXd& s_coeffsV6, const Eigen::VectorXd& d_coeffsV6, double t);

	// Calculates polynomial value at time 't' and returns 3-dim vector: [s, s_d, s_dd]
	static Eigen::VectorXd calc_polynomial_at (const Eigen::VectorXd& coeffsX6, double t);
	// Returns Jerk value using polynomial value at time 't'.
	inline double calc_polynomial_jerk_at (const Eigen::VectorXd& coeffsX6, double t) const;
	inline double calc_polynomial_accel_at(const Eigen::VectorXd& coeffsX6, double t) const;
	inline double calc_polynomial_velocity_at (const Eigen::VectorXd& coeffsX6, double t) const;

	double calcSafetyDistanceCost(double Sdist, double Ddist, double velocity) const;
	double calcLongitudialSafetyDistanceCost(double Sdist, double velocity) const;
	double calcLateralSafetyDistanceCost(double Ddist) const;

	// Calculate Jerk optimal polynomial for S-trajectory with keeping velocity, using current S start-state [s, s_d, s_dd], 
	// target velocity m/s and specified duration T in sec.
	// Returns: polynomial copeffs as 6-dim vector.
	static Eigen::VectorXd calc_s_polynomial_velocity_keeping(const Eigen::VectorXd& startStateV3, double targetVelocity, double T);

private:
	double timeStart_;
	double duration_;		// duration T
	double dt_ = delta_t;	// time step along a trajectory
	double cost_dt_ = 0.1;  // time step along a trajectory used during evaluation of trajectory-cost.

	Eigen::VectorXd cost_{ Eigen::VectorXd::Zero(4) };

	Eigen::VectorXd startState_; // [s, s_dot, s_ddot, d, d_dot, d_ddot]
	Eigen::VectorXd endState_;   // [s, s_dot, s_ddot, d, d_dot, d_ddot]
	Eigen::VectorXd S_coeffs_;   // 6 coeffs of quintic polynomial
	Eigen::VectorXd D_coeffs_;   // 6 coeffs of quintic polynomial

public:
	// DEBUGING part
	double DE_V = 0;

	mutable double DE_DIF_S = 0;
	mutable double DE_DIF_D = 0;
};


/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

inline double Trajectory::calc_polynomial_jerk_at(const Eigen::VectorXd& a, double t) const
{
	// s(t) = a0 + a1 * t + a2 * t^2 + a3 * t^3 + a4 * t^4 + a5 * t^5
	// s_d(t) = a1 + 2*a2 * t + 3*a3 * t^2 + 4*a4 * t^3 + 5*a5 * t^4
	// s_dd(t) = 2*a2 + 6*a3 * t^1 + 12*a4 * t^2 + 20*a5 * t^3

	// calculate s_ddd(t)
	return 6 * a(3) + 24 * a(4) * t + 60 * a(5) * t * t;
}

inline double Trajectory::calc_polynomial_accel_at(const Eigen::VectorXd& a, double t) const
{
	// s(t) = a0 + a1 * t + a2 * t^2 + a3 * t^3 + a4 * t^4 + a5 * t^5
	// s_d(t) = a1 + 2*a2 * t + 3*a3 * t^2 + 4*a4 * t^3 + 5*a5 * t^4
	// s_dd(t) = 2*a2 + 6*a3 * t^1 + 12*a4 * t^2 + 20*a5 * t^3

	const auto t2 = t * t;
	const auto t3 = t2 * t;

	// calculate s_dd(t)
	return 2 * a(2) + 6 * a(3) * t + 12 * a(4) * t2 + 20 * a(5) * t3;
}

inline double Trajectory::calc_polynomial_velocity_at(const Eigen::VectorXd& a, double t) const
{
	// s(t) = a0 + a1 * t + a2 * t^2 + a3 * t^3 + a4 * t^4 + a5 * t^5
	// s_d(t) = a1 + 2*a2 * t + 3*a3 * t^2 + 4*a4 * t^3 + 5*a5 * t^4
	// s_dd(t) = 2*a2 + 6*a3 * t^1 + 12*a4 * t^2 + 20*a5 * t^3

	const auto t2 = t * t;
	const auto t3 = t2 * t;
	const auto t4 = t3 * t;

	// calculate s_d(t)
	return a(1) + 2. * a(2) * t + 3. * a(3) * t2 + 4. * a(4) * t3 + 5. * a(5) * t4;
}

/*
inline double logistic(double x)
{
	return 2. / (1 + exp(-x)) - 1.;
}*/


inline double Trajectory::calcLongitudialSafetyDistanceCost(double Sdist, double velocity) const
{
	//const auto safetyDist = Utils::braking_distance(velocity);
	constexpr double D0 = 2; // safety distance, known as constant time gap law.
	constexpr double tau = 1.8;
	const auto safetyDist = D0 + tau * velocity;

	if (Sdist > safetyDist)
		return 0.0;

	// https://www.desmos.com/calculator/q2akbfqxkb
	return std::exp(Sdist * std::log(0.08) / (safetyDist - D0));
}

inline double Trajectory::calcLateralSafetyDistanceCost(double Ddist) const
{
	constexpr double safetyDist = 1.5;
	if (Ddist > 1.25 * safetyDist)
	{
		return 0;
	}
	return std::exp(-Ddist / (-safetyDist / std::log(0.1)));
}

inline double Trajectory::calcSafetyDistanceCost(double Sdist, double Ddist, double velocity) const
{
	return calcLongitudialSafetyDistanceCost(Sdist, velocity);

	const double longitudinalDistCost = calcLongitudialSafetyDistanceCost(Sdist, velocity);
	const double lateralDistCost = calcLateralSafetyDistanceCost(Ddist);

	if (lateralDistCost > 1.5)
	{
		return 0;
	}
	else if (longitudinalDistCost < 2)
	{
		return std::max(longitudinalDistCost, lateralDistCost);
	}
	
	return longitudinalDistCost;
}
