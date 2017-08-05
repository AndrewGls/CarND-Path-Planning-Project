#pragma once

#include <vector>
#include "Eigen-3.3/Eigen/Core"


class Trajectory
{
public:
	Trajectory();

	// Input:
	//   startStateX6 is a vector of size 6: [s, s_d, s_dd, d, d_d, d_dd]
	//   endStateX6 is a vector of size 6:   [s, s_d, s_dd, d, d_d, d_dd]
	Trajectory(const Eigen::VectorXd& startStateX6, const Eigen::VectorXd& endStateX6, double duration, double timeStart);

	// Returns array of vectors with time step, where each vector is sd-point [s, s_d, s_dd, d, d_d, d_dd]
	std::vector<Eigen::VectorXd> getTrajectorySDPoints(double dt);

	// Returns vector state [s, s_d, s_dd, d, d_d, d_dd] at the time 'time'.
	Eigen::VectorXd evalaluateStateAt(double time) const;

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
	static Eigen::VectorXd calc_polynomial_at(const Eigen::VectorXd& coeffsX6, double t);

private:
	double timeStart_;
	double duration_;	// duration T
	double cost_;
	double dt_ = 0.02;	// time step along a trajectory

	Eigen::VectorXd startState_; // [s, s_dot, s_ddot, d, d_dot, d_ddot]
	Eigen::VectorXd endState_;   // [s, s_dot, s_ddot, d, d_dot, d_ddot]
	Eigen::VectorXd S_coeffs_;   // 6 coeffs of quintic polynomial
	Eigen::VectorXd D_coeffs_;   // 6 coeffs of quintic polynomial
};