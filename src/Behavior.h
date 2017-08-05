#pragma once

#include "Types.h"
#include "TrajectoryPool.h"
#include "SensorFusion.h"
#include "VehicleState.h"
#include <vector>
#include <memory>


class Behavior
{
public:
	Behavior();

	// currStateX6 is a vector of size 6: [s, s_d, s_dd, d, d_d, d_dd]
	TrajectoryPtr optimalTrajectory(const Eigen::VectorXd& currStateX6, double currTime, const SensorFusion& sensorFusion);

private:
	TVehicleStates m_vState;
	using VehicleStatePtr = std::unique_ptr<CVehicleState>;
	std::vector<VehicleStatePtr> m_scenarios;

	const double speed_limit_ = (50. - 2.) * 0.44704; // speed limit in m/s
};
