#include "Behavior.h"
#include "LineKeeping.h"
#include <iostream>
#include <assert.h>


using Eigen::VectorXd;
using Eigen::MatrixXd;

using namespace std;

Behavior::Behavior()
{
	m_states.reserve(2);
	m_states.push_back(std::make_unique<LineKeeping>());
}

TrajectoryPtr Behavior::optimalTrajectory (const VectorXd& currStateX6, double currTime, const SensorFusion& sensorFusion)
{
	assert(m_states.size() < 3);

	VehicleState* pNewState = nullptr;
	TrajectoryPtr pOptimalTrajectory;
	VehicleState* pCurrState = m_states.back().get();

	tie(pNewState, pOptimalTrajectory) = pCurrState->optimalTrajectory(currStateX6, currTime, sensorFusion);

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
