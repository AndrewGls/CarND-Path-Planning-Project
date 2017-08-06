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

private:
	static constexpr double JerkCostW = 0.01;
	static constexpr double VelocityCostW = 1.;
	static constexpr double TimeCostW = 100;
};