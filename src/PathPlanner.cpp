#include "PathPlanner.h"
#include "LaneKeeping.h"
#include <iostream>
#include <assert.h>
#include <memory>


using Eigen::VectorXd;
using Eigen::MatrixXd;

using namespace std;

PathPlanner::PathPlanner()
{
	m_states.reserve(2);
	m_states.push_back(unique_ptr<LineKeeping>(new LineKeeping));
}

TrajectoryPtr PathPlanner::OptimalTrajectory (const VectorXd& currStateX6, double currTime, const SensorFusion& sensorFusion)
{
	assert(m_states.size() < 3);

	VehicleState* pNewState = nullptr;
	TrajectoryPtr pOptimalTrajectory;
	VehicleState* pCurrState = m_states.back().get();

	tie(pNewState, pOptimalTrajectory) = pCurrState->OptimalTrajectory(currStateX6, currTime, sensorFusion);

	if (!pNewState)
	{
#ifdef VERBOSE_STATE
		std::cout << " Removing Change Lane State " << std::endl;
#endif
		assert(m_states.size() == 2);
		m_states.pop_back();
	}
	else if (pNewState != pCurrState)
	{
#ifdef VERBOSE_STATE
		std::cout << " Adding Change Lane State " << std::endl;
#endif
		assert(m_states.size() == 1);
		m_states.push_back(VehicleStatePtr(pNewState));
	}

	return pOptimalTrajectory;
}
