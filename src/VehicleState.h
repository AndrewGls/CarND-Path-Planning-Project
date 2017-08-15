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

	virtual std::tuple<VehicleState*, TrajectoryPtr> OptimalTrajectory(const Eigen::VectorXd& currStateX6, double currTime, const SensorFusion& sensFusion) = 0;

protected:
	double GetCorrectedVelocity(double velocity, int nLane) const;

protected:
	constexpr static double m_SpeedLimit = (50 - 2) * 0.44704; // speed limit in m/s
	constexpr static double m_HorizontPrediction = 10; // sec
	constexpr static double m_TimeStep = 0.1;
};


inline double VehicleState::GetCorrectedVelocity(double velocity, int nLane) const
{
	// different lanes has different MAX SPEED!!!!
	static const double m_MaxSpeedFactors[3] = { 0.95, 0.975, 1. };
	assert(nLane >= 0 && nLane <= 2);
	return velocity * m_MaxSpeedFactors[nLane];
}
