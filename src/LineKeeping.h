#pragma once

#include "VehicleState.h"

class LineKeeping : public CVehicleState
{
public:
	LineKeeping();
	~LineKeeping();

	virtual TrajectoryPtr optimalTrajectory(const Eigen::VectorXd& currStateX6,
											double currTime,
											const SensorFusion& sensFusion) override;

};