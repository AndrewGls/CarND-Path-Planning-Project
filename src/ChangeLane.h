#pragma once
#ifndef CHANGE_LANE_H
#define CHANGE_LANE_H

#include "VehicleState.h"

class ChangeLane : public VehicleState
{
public:
	ChangeLane() {}

	virtual std::tuple<VehicleState*, TrajectoryPtr> optimalTrajectory(
			const Eigen::VectorXd& currStateX6, double currTime, const SensorFusion& sensFusion) override;
};

#endif // CHANGE_LANE_H
