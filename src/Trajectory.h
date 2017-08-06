#pragma once

#include "Types.h"
#include <vector>
#include <memory>
#include <utility>


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
	static TrajectoryPtr VelocityKeeping_STrajectory (const Eigen::VectorXd& currStateX6, double targetVelocity, double currTime, double timeDuration);

	// Returns array of vectors with time step, where each vector is sd-point [s, s_d, s_dd, d, d_d, d_dd]
	std::vector<Eigen::VectorXd> getTrajectorySDPoints (double dt);
	void getTrajectorySDPointsTo (std::vector<Eigen::VectorXd>& outPoints, double dt);

	// Returns vector state [s, s_d, s_dd, d, d_d, d_dd] at the time 'time'.
	Eigen::VectorXd evalaluateStateAt(double time) const;

	// Returns trajectory duration in seconds.
	double getDuration() const { return duration_; }

	double addCost(double value) { cost_ += value; return cost_; }
	double getCost() const { return cost_; }

	// Calculation of cost functions.
	// See: https://d17h27t6h515a5.cloudfront.net/topher/2017/July/595fd482_werling-optimal-trajectory-generation-for-dynamic-street-scenarios-in-a-frenet-frame/werling-optimal-trajectory-generation-for-dynamic-street-scenarios-in-a-frenet-frame.pdf

	// Calculate Jerk cost function for evaluated trajectory points as array of vectors like [s, s_d, s_dd, d, d_d, d_dd].
	double jerkCost_SD();
	double jerkCost_SD(double timeStep);

	std::pair<double, double> MinMaxVelocity_S();
	std::pair<double, double> MinMaxVelocity_S(double timeStep);

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
	inline double calc_polynomial_velocity_at (const Eigen::VectorXd& coeffsX6, double t) const;


	// Calculate Jerk optimal polynomial for S-trajectory with keeping velocity, using current S start-state [s, s_d, s_dd], 
	// target velocity m/s and specified duration T in sec.
	// Returns: polynomial copeffs as 6-dim vector.
	static Eigen::VectorXd calc_s_polynomial_velocity_keeping(const Eigen::VectorXd& startStateV3, double targetVelocity, double T);

private:
	double timeStart_ = 0;
	double duration_ = 0;	// duration T
	double cost_ = 0;
	double dt_ = delta_t;	// time step along a trajectory

	Eigen::VectorXd startState_; // [s, s_dot, s_ddot, d, d_dot, d_ddot]
//	Eigen::VectorXd endState_;   // [s, s_dot, s_ddot, d, d_dot, d_ddot]
	Eigen::VectorXd S_coeffs_;   // 6 coeffs of quintic polynomial
	Eigen::VectorXd D_coeffs_;   // 6 coeffs of quintic polynomial

public:
	// DEBUGING part
	double DE_V = 0;
};


/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

inline double Trajectory::calc_polynomial_jerk_at(const Eigen::VectorXd& a, double t) const
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
	return a(1) + 2 * a(2) * t + 3 * a(3) * t2 + 4 * a(4) * t3 + 5 * a(5) * t4;
}

