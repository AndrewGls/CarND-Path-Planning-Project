#pragma once

#include "Trajectory.h"
#include <deque>
#include <memory>
#include "Eigen-3.3/Eigen/Core"


class TrajectoryPool
{
public:
	using TPool = std::deque<TrajectoryPtr>;

	TrajectoryPool() {}

	void add(TrajectoryPtr traj) { m_pool.push_back(traj); }
	const TPool getPool() const { return m_pool; }
	TPool getPool() { return m_pool; }

private:
	TPool m_pool;
};
