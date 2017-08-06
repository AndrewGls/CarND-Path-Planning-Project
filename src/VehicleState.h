#pragma once

#include "Types.h"
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

	double m_delta_t = delta_t;	// default time step along a trajectory
	constexpr static double m_SpeedLimit = 22;// (50. - 2.) * 0.44704; // speed limit in m/s
	constexpr static double m_SafetyDist = 0.5; // in meters, constant time gap law - the safety distance to the target car.
	constexpr static double m_MaxJerkS = 20;


	constexpr static double saferyDistance (double speed) { return m_SafetyDist + 1.5 * speed; }

	virtual TrajectoryPtr optimalTrajectory(const Eigen::VectorXd& currStateX6, double currTime, const SensorFusion& sensFusion) = 0;

};
