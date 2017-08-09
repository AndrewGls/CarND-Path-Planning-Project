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


#if 0
#include <random>
#include <iostream>

inline int FKTest()
{
	double s = 10;
	double d = 2;
	double vs = 12;
	double vd = 1;

	int n = 20;
	double t = 0;
	double dt = 0.5;

	std::default_random_engine gen(13);
	std::normal_distribution<double> dist_vs(0, 2);
	std::normal_distribution<double> dist_vd(0, 0.2);

	std::vector<std::vector<double>> S;
	std::vector<std::vector<double>> Sn;
	std::vector<std::vector<double>> Out;

	for (int i = 0; i < n; i++)
	{
		std::vector<double> x(3, 0);

		x[0] = s + vs * t;
		x[1] = d + vd * t;
		x[2] = vs;

		S.push_back(x);

		double vs_n = (vs + dist_vs(gen));
		double vd_n = (vd + dist_vd(gen));

		x[0] = s + vs_n * t;
		x[1] = d + vd_n * t;
		x[2] = vs_n;

		Sn.push_back(x);

		t += dt;
	}

	KalmanFilter kf(0, 0, 0);

	t = 0;

	for (int i = 0; i < n; i++)
	{
		kf.UpdateStep(Sn[i][0], Sn[i][1], Sn[i][2]);
		kf.PredictStep(dt);

		std::vector<double> x{ kf.s(), kf.d(), kf.vs() };
		Out.push_back(x);
	}


	std::cout << "-------- ORG ------------" << std::endl;

	for (int i = 0; i < S.size(); i++) {
		std::cout << S[i][0] << ", " << S[i][1] << ", " << S[i][2] << std::endl;
	}

	std::cout << "---------N added--------------" << std::endl;

	for (int i = 0; i < Sn.size(); i++) {
		std::cout << Sn[i][0] << ", " << Sn[i][1] << ", " << Sn[i][2] << std::endl;
	}

	std::cout << "---------Filtered--------------" << std::endl;

	for (int i = 0; i < Out.size(); i++) {
		std::cout << Out[i][0] << ", " << Out[i][1] << ", " << Out[i][2] << std::endl;
	}

	return 1;
}

static int i = FKTest();
#endif