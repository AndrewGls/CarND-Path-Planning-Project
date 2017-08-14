#pragma once

#include "Types.h"
#include "TrajectoryPool.h"
#include "SensorFusion.h"
#include "VehicleState.h"
#include <vector>
#include <memory>


class PathPlanner
{
public:
	PathPlanner();

	// currStateX6 is a vector of size 6: [s, s_d, s_dd, d, d_d, d_dd]
	TrajectoryPtr OptimalTrajectory(const Eigen::VectorXd& currStateX6, double currTime, const SensorFusion& sensorFusion);

private:
	using VehicleStatePtr = std::unique_ptr<VehicleState>;
	std::vector<VehicleStatePtr> m_states;
};
