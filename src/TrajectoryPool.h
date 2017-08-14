#pragma once

#include "OtherVehicle.h"
#include "Trajectory.h"
#include "Eigen-3.3/Eigen/Core"
#include <deque>
#include <memory>
#include <limits>


class TrajectoryPool
{
public:
	using TPool = std::deque<TrajectoryPtr>;

	TrajectoryPool(double speedLimit, double timeHorizon);

	void SetOtherCars(const TOtherCarsTrajectory& otherTrajectories);
	void AddOtherCars(const TOtherCarsTrajectory& otherTrajectories);
	void AddTrajectory(TrajectoryPtr traj);

	TrajectoryPtr OptimalTrajectory() const;

private:
	void CalcTrajectoryCost(TrajectoryPtr traj);

private:
	TPool m_pool;
	TOtherCarsTrajectory m_otherTrajectories;

	double m_SpeedLimit;
	double m_HorizontPrediction;

	constexpr static double m_MaxJerkS = 9;

	static constexpr double JerkCostWeight = 0.2;// 0.5;// 0.01;
	static constexpr double AccelCostWeight = 1;
	static constexpr double VelocityCostWeight = 0.5;
	static constexpr double SaferyDistCostWeight = 1000;
	static constexpr double LaneOffsetCostWeight = 0.5;
};
