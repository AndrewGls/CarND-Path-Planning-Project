#pragma once

#include "Types.h"
#include "Trajectory.h"
#include "SensorFusion.h"
#include "OtherVehicle.h"
#include <vector>
#include <tuple>


class VehicleState
{
public:
	virtual ~VehicleState() {}

	virtual std::tuple<VehicleState*, TrajectoryPtr> optimalTrajectory(const Eigen::VectorXd& currStateX6, double currTime, const SensorFusion& sensFusion) = 0;

protected:
	constexpr static double m_SpeedLimit = (50 - 2) * 0.44704; // speed limit in m/s
	constexpr static double m_HorizontPrediction = 10; // sec
	constexpr static double m_TimeStep = 0.1;
};
