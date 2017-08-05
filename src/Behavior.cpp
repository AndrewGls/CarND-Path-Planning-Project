#include "Behavior.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;


Behavior::Behavior()
{
}

const Trajectory* Behavior::generateTrajectory(const VectorXd& currStateX6, double currTime, const SensorFusion& sensorFusion)
{
	double T = 30;

	VectorXd s_state(6);
	VectorXd f_state(6);
	s_state << 125, 0, 0, -6, 0, 0;
	f_state << 125+50, 20, 0, -6, 0, 0;
	control_.generateOptimalTrajectory(s_state, f_state, T, currTime);


/*	double s_i = currStateX6(0);
	double v_i = currStateX6(1);
	double a_i = currStateX6(2);
	double d_i = currStateX6(3);

	double s_f = s_i + 50;
	double v_f = 20;
	double a_f = 0;
	double d_f = d_i;

	VectorXd finalStateX6(6);
	finalStateX6 << s_f, v_f, a_f, d_i, 0, 0;

	control_.generateOptimalTrajectory(currStateX6, finalStateX6, T, currTime);
*/
	Eigen::VectorXd stateI = control_.getOptimalTragectory()->evalaluateStateAt(0);
	Eigen::VectorXd stateF = control_.getOptimalTragectory()->evalaluateStateAt(T);
	Eigen::VectorXd state02 = control_.getOptimalTragectory()->evalaluateStateAt(0.02);
	Eigen::VectorXd state08 = control_.getOptimalTragectory()->evalaluateStateAt(T - 0.02);
	Eigen::VectorXd state05 = control_.getOptimalTragectory()->evalaluateStateAt(T/2);

	std::cout << stateI(0) << ", ";
	std::cout << stateI(1) << ", ";
	std::cout << stateI(2);
	std::cout << std::endl;

	std::cout << stateF(0) << ", ";
	std::cout << stateF(1) << ", ";
	std::cout << stateF(2);
	std::cout << std::endl;

	std::cout << state02(0) << ", ";
	std::cout << state02(1) << ", ";
	std::cout << state02(2);
	std::cout << std::endl;

	std::cout << state08(0) << ", ";
	std::cout << state08(1) << ", ";
	std::cout << state08(2);
	std::cout << std::endl;

	std::cout << state05(0) << ", ";
	std::cout << state05(1) << ", ";
	std::cout << state05(2);
	std::cout << std::endl;

	return control_.getOptimalTragectory();
}
