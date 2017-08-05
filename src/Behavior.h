#pragma once

#include "Types.h"
#include "TrajectoryController.h"
#include "SensorFusion.h"

class Behavior
{
public:
	Behavior();

	// currStateX6 is a vector of size 6: [s, s_d, s_dd, d, d_d, d_dd]
	const Trajectory* generateTrajectory(const Eigen::VectorXd& currStateX6, double currTime, const SensorFusion& sensorFusion);

private:
	TrajectoryController control_;

	const double speed_limit_ = (50. - 2.) * 0.44704; // speed limit in m/s
};
