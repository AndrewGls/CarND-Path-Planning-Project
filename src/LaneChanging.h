#pragma once
#ifndef CHANGE_LANE_H
#define CHANGE_LANE_H

#include "VehicleState.h"

class LaneChanging : public VehicleState
{
public:
	LaneChanging(int nStartLane, int nTargetLane);

	virtual std::tuple<VehicleState*, TrajectoryPtr> OptimalTrajectory(
			const Eigen::VectorXd& currStateX6, double currTime, const SensorFusion& sensFusion) override;

private:
	int m_nStartLane;
	int m_nTargetLane;
};

#endif // CHANGE_LANE_H
