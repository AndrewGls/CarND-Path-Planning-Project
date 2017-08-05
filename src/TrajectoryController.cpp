#include "TrajectoryController.h"


TrajectoryController::TrajectoryController()
{
	trajs_.resize(numGenTrajs_);
}

const Trajectory* TrajectoryController::generateOptimalTrajectory(const Eigen::VectorXd& startStateX6,
																  const Eigen::VectorXd& endStateX6,
																  double duration,
																  double timeStart)

{
	TrajectoryPtr traj = std::make_unique<Trajectory>(startStateX6, endStateX6, duration, timeStart);
	optimalTraj_ = std::move(traj);

	return optimalTraj_.get();
}

