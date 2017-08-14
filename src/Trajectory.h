#pragma once

#include "Types.h"
#include "Utils.hpp"
#include <vector>
#include <memory>
#include <utility>
#include <cmath>
#include <iostream>


class Trajectory;
using TrajectoryPtr = std::shared_ptr<Trajectory>;


class Trajectory
{
public:
	Trajectory();

	// Input:
	//   startStateX6 is a vector of size 6: [s, s_d, s_dd, d, d_d, d_dd]
	//   endStateX6 is a vector of size 6:   [s, s_d, s_dd, d, d_d, d_dd]
	Trajectory(const Eigen::VectorXd& startStateX6, const Eigen::VectorXd& endStateX6, double durationS, double durationD, double timeStart);

	// Calculates Jerk Minimizing Trajectory for the Velocity Leeping state, using currect state, target velocity and duration T.
	// currStateX6 is 6-dim vector: [s, s_d, s_dd, d, d_d, d_dd].
	// Usually only the first 3 coord. [s, s_d, s_dd] is used for trajectory calculation (it is better to split to S-trajectory and D-trajectory).
	// https://d17h27t6h515a5.cloudfront.net/topher/2017/July/595fd482_werling-optimal-trajectory-generation-for-dynamic-street-scenarios-in-a-frenet-frame/werling-optimal-trajectory-generation-for-dynamic-street-scenarios-in-a-frenet-frame.pdf
	static TrajectoryPtr VelocityKeeping_STrajectory (const Eigen::VectorXd& currStateX6, double endD, double targetVelocity, double currTime, double timeDurationS, double timeDurationD);

	// Returns vector state [s, s_d, s_dd, d, d_d, d_dd] at the time 'time'.
	Eigen::VectorXd EvaluateStateAt(double time) const;

	// Calculation of cost functions.
	// See: https://d17h27t6h515a5.cloudfront.net/topher/2017/July/595fd482_werling-optimal-trajectory-generation-for-dynamic-street-scenarios-in-a-frenet-frame/werling-optimal-trajectory-generation-for-dynamic-street-scenarios-in-a-frenet-frame.pdf

	// Calculate Jerk cost function for evaluated trajectory points as array of vectors like [s, s_d, s_dd, d, d_d, d_dd].
	double CalcJerkCost(double timeDuration, double& maxJs, double& maxJd) const;
	double CalcAccelCost(double timeDuration) const;
	double CalcVelocityCost(double targetVelocity, double timeDuration) const;
	double CalcSaferyDistanceCost(const Eigen::MatrixXd& s2, double timeDuration) const;
	double CalcLaneOffsetCost(double timeDuration) const;
	std::pair<double, double> MinMaxVelocity_S() const;

	// Returns trajectory duration in seconds.
	double GetDurationS() const { return m_durationS; }
	double GetDurationD() const { return m_durationD; }
	Eigen::VectorXd GetStartState() const { return m_startState; }

	double GetTargetS() const { return m_endState(0); }
	double GetTargetD() const { return m_endState(3); }
	double GetStartD() const { return m_startState(3); }

	double GetTotalCost() const { return m_cost.sum(); }

	void SetJerkCost (double cost)			{ m_cost(0) = cost; }
	void SetAccelCost(double cost)			{ m_cost(1) = cost; }
	void SetVelocityCost (double cost)		{ m_cost(2) = cost; }
	void SetSafetyDistanceCost (double cost)	{ m_cost(3) = cost; }
	void SetLaneOffsetCost(double cost)		{ m_cost(4) = cost; }

	double GetJerkCost() const			{ return m_cost(0); }
	double GetAccelCost() const			{ return m_cost(1); }
	double GetVelocityCost() const		{ return m_cost(2); }
	double GetSafetyDistanceCost() const { return m_cost(3); }
	double GetLaneOffsetCost() const	{ return m_cost(4); }

	double TragetVelocity() const { return m_tragetVelocity; }
	void PrintInfo();

private:
	double CalcSafetyDistanceCost(double Sdist, double Ddist, double velocity) const;

	// Calculates Jerk Minimizing Trajectory for start state, end state and duration T.
	// Input:
	//   startStateX3 is vector of size 3: [s, s_d, s_dd]
	//   endStateX3 is vector of size 3:   [s, s_d, s_dd]
	// Returns:
	//   vector of size 6: [s, s_d, s_dd, a3, a4, a5]
	static Eigen::VectorXd CalcQuinticPolynomialCoeffs(const Eigen::VectorXd& startStateX3, const Eigen::VectorXd& endStateX3, double T);

	// Calculates polynomial value at time 't' and returns 3-dim vector: [s, s_d, s_dd]
	static Eigen::VectorXd calc_polynomial_at (const Eigen::VectorXd& coeffsX6, double t);
	// Returns Jerk value using polynomial value at time 't'.
	inline double calc_polynomial_jerk_at (const Eigen::VectorXd& coeffsX6, double t) const;
	inline double calc_polynomial_accel_at(const Eigen::VectorXd& coeffsX6, double t) const;
	inline double calc_polynomial_velocity_at (const Eigen::VectorXd& coeffsX6, double t) const;
	inline double calc_polynomial_distance_at(const Eigen::VectorXd& coeffsX6, double t) const;

	// Calculate Jerk optimal polynomial for S-trajectory with keeping velocity, using current S start-state [s, s_d, s_dd], 
	// target velocity m/s and specified duration T in sec.
	// Returns: polynomial copeffs as 6-dim vector.
	static Eigen::VectorXd calc_s_polynomial_velocity_keeping(const Eigen::VectorXd& startStateV3, double targetVelocity, double T);

private:
	double m_timeStart;
	double m_durationS;		// duration T of S state in secs
	double m_durationD;		// duration T of D state in secs
	double m_dt = Utils::delta_t;	// time step along a trajectory
	double m_costDT = 0.1;  // time step along a trajectory used during evaluation of trajectory-cost.
	double m_tragetVelocity;

	Eigen::VectorXd m_cost{ Eigen::VectorXd::Zero(5) };

	Eigen::VectorXd m_startState; // [s, s_dot, s_ddot, d, d_dot, d_ddot]
	Eigen::VectorXd m_endState;   // [s, s_dot, s_ddot, d, d_dot, d_ddot]
	Eigen::VectorXd m_Scoeffs;   // 6 coeffs of quintic polynomial
	Eigen::VectorXd m_Dcoeffs;   // 6 coeffs of quintic polynomial
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

inline double Trajectory::calc_polynomial_distance_at(const Eigen::VectorXd& a, double t) const
{
	// s(t) = a0 + a1 * t + a2 * t^2 + a3 * t^3 + a4 * t^4 + a5 * t^5
	// s_d(t) = a1 + 2*a2 * t + 3*a3 * t^2 + 4*a4 * t^3 + 5*a5 * t^4
	// s_dd(t) = 2*a2 + 6*a3 * t^1 + 12*a4 * t^2 + 20*a5 * t^3

	const auto t2 = t * t;
	const auto t3 = t2 * t;
	const auto t4 = t3 * t;
	const auto t5 = t4 * t;

	// calculate s_d(t)
	return a(0) + a(1) * t + a(2) * t2 + a(3) * t3 + a(4) * t4 + a(5) * t5;
}
