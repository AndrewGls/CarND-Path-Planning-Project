#pragma once

#include "Trajectory.h"
#include <deque>
#include <memory>
#include "Eigen-3.3/Eigen/Core"
#include <limits>


using TOtherCarsTrajectory = std::vector<Eigen::MatrixXd>;


class TrajectoryPool
{
public:
	using TPool = std::deque<TrajectoryPtr>;

	TrajectoryPool(const TOtherCarsTrajectory& otherTrajectories, double speedLimit, double timeHorizon);

	void addTrajectory(TrajectoryPtr traj);
	TrajectoryPtr optimalTrajectory() const;

	const TPool getPool() const { return m_pool; }
	TPool getPool() { return m_pool; }

private:
	void calcTrajectoryCost(TrajectoryPtr traj);

private:
	TPool m_pool;
	TOtherCarsTrajectory m_otherTrajectories;

	double m_SpeedLimit = 22;// (50. - 2.) * 0.44704; // speed limit in m/s
	double m_HorizontPrediction = 4.;//3;// 1.; // sec

	constexpr static double m_MaxJerkS = 100;

	static constexpr double JerkCostWeight = 0.5;// 0.01;
	static constexpr double VelocityCostWeight = 0.5;
	static constexpr double TimeCostWeight = 0;// 100;
	static constexpr double SaferyDistCostWeight = 1000;
};
