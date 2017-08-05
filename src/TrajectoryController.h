#pragma once

#include "Trajectory.h"
#include <vector>
#include <memory>
#include "Eigen-3.3/Eigen/Core"


class TrajectoryController
{
public:
	TrajectoryController();

	const Trajectory* generateOptimalTrajectory(const Eigen::VectorXd& startStateX6, const Eigen::VectorXd& endStateX6, double duration, double timeStart);
	const Trajectory* getOptimalTragectory() const { return optimalTraj_.get(); }

private:
	const int numGenTrajs_ = 1;

	using TrajectoryPtr = std::unique_ptr<Trajectory>;
	std::vector<TrajectoryPtr> trajs_;

	TrajectoryPtr optimalTraj_;
};
