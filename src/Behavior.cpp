#include <iostream>
#include "Behavior.h"
#include "LineKeeping.h"


using Eigen::VectorXd;
using Eigen::MatrixXd;

using namespace std;

Behavior::Behavior()
	: m_vState(LineKeepingState)
	, m_scenarios(NumOfStates)
{
	m_scenarios[LineKeepingState] = std::make_unique<LineKeeping>();
}

TrajectoryPtr Behavior::optimalTrajectory (const VectorXd& currStateX6, double currTime, const SensorFusion& sensorFusion)
{
	return m_scenarios[m_vState]->optimalTrajectory(currStateX6, currTime, sensorFusion);
}
