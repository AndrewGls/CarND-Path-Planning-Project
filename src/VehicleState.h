#pragma once

#include "Trajectory.h"
#include "SensorFusion.h"


enum TVehicleStates
{
	LineKeepingState,

	NumOfStates = LineKeepingState + 1
};



class CVehicleState
{
public:
	virtual ~CVehicleState() {}

	virtual TrajectoryPtr optimalTrajectory(const Eigen::VectorXd& currStateX6, double currTime, const SensorFusion& sensFusion) = 0;

};
