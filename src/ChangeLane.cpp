#include "ChangeLane.h"

using namespace std;


tuple<VehicleState*, TrajectoryPtr> ChangeLane::optimalTrajectory(const Eigen::VectorXd& currStateX6,
											double currTime,
											const SensorFusion& sensFusion)
{
	return tuple<VehicleState*, TrajectoryPtr>(this, nullptr);
}
