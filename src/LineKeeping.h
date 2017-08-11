#pragma once

#include "VehicleState.h"

class LineKeeping : public VehicleState
{
public:
	LineKeeping() {}

	virtual std::tuple<VehicleState*, TrajectoryPtr> optimalTrajectory(
		const Eigen::VectorXd& currStateX6, double currTime, const SensorFusion& sensFusion) override;

};