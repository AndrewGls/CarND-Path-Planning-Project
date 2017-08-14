#pragma once

#include "Types.h"
#include <stack>


// See Pedestrian traking:
//		Term 2, Lesson 5: "Lidar and Radar Fusion with Kalman Filter": "8. State Prediction" and "12. Laser Measurements Part 3".

// Predicted:
// x = [s, d, s_v, d_v]^t

// Model:
// s'(t) = s + v_s * t
// d'(t) = d + v_d * t
// v_s'(t) = v_s
// v_d'(t) = v_d

// Prediction Step:
//
// x' = F * x + noise, where noise=0
// P' = F * P * F^t + Q
//
// For Q see Term_2, Lesson_5: Laser Measurements Part 3.
//
//     |1 0 t 0|        |var_s  0    0      0  |       |dt^4*var_ax/4          0         dt^3*var_ax/2         0      |
// F = |0 1 0 t|    P = |0   var_d   0      0  |   Q = |     0            dt^4*var_ay/4        0         dt^3*var_ay/2|
//     |0 0 1 0|        |0      0  var_vs   0  |       |dt^3*var_ax^2/2        0         dt^2*var_ax           0      |
//     |0 0 0 0|        |0      0    0   var_vd|       |     0            dt^3*var_ay/2        0          dt^2*var_ay |
//
//

// Measurement Update Step:
// 
// z = [s, d, vs]^t
// 
// y = z - H * x
// S = H * P * H^t + R
// K = P * H^t * S^-1
//
// new state:
// x = x + (K * y)
// P = (I - K * H) * P
//
// Where  H = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]]
//

// This version of Kalman filer ignores vd becuase other cars usualy drives along one lane: z = [s, d, vs] instead of z = [s, d, vs, vd].
// If it needs, replace z = [s, d, vs] to z = [s, d, vs, vd] with small modifications in Kalman Filter.
class KalmanFilter
{
public:
	KalmanFilter(double s = 0, double d = 0, double vs = 0);

	Eigen::VectorXd PredictStep(double dt);
	void UpdateStep(double s, double d, double vs);

	Eigen::VectorXd x() const { return x_; }
	double s() const { return x_(0); }
	double d() const { return x_(1); }
	double vs() const { return x_(2); }

	void SaveState();
	void RestoreState();

private:
	void update_Q(double dt);

private:
	Eigen::VectorXd x_; // 4x1
	Eigen::MatrixXd F_; // 4x4
	Eigen::MatrixXd P_; // 4x4
	Eigen::MatrixXd Q_; // 4x4, State Covariance Matrix is defined by Acceleration Noise Parameters.

	Eigen::MatrixXd H_; // 3x4
	Eigen::MatrixXd R_; // 3x3, Measurement Noise Covariance Matrix which represents the uncertainty in our sensor measurements.

	std::stack<Eigen::VectorXd> x_stack_;
	std::stack<Eigen::MatrixXd> P_stack_;
};

