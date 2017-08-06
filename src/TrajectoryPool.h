#pragma once

#include "Trajectory.h"
#include <deque>
#include <memory>
#include "Eigen-3.3/Eigen/Core"
#include <limits>

class TrajectoryPool
{
public:
	using TPool = std::deque<TrajectoryPtr>;

	TrajectoryPool() {}

	void add(TrajectoryPtr traj) { m_pool.push_back(traj); }
	TrajectoryPtr optimalTrajectory() const;

	const TPool getPool() const { return m_pool; }
	TPool getPool() { return m_pool; }

private:
	TPool m_pool;
};


//---------------------------------------------------------------------------------------

inline TrajectoryPtr TrajectoryPool::optimalTrajectory() const
{
	TrajectoryPtr pTraj;
	double minCost = 1e10;// std::numeric_limits<double>::max();

	for (auto it = m_pool.cbegin(); it != m_pool.cend(); ++it)
	{
		const auto cost = (*it)->getCost();
		if (cost < minCost)
		{
			minCost = cost;
			pTraj = *it;
		}
	}

	return pTraj;
}

