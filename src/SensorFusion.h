#pragma once

#include "Types.h"
#include "Waypoints.h"
#include "OtherVehicle.h"
#include <unordered_map>


// Helper class to Predict position of vehicles after delayTime.
class SensorFusion
{
public:
	SensorFusion() {}

	void Predict(double deltaTime);
	void Update (const std::vector<SensorFusionData>& sensorFusion, double currS, const Waypoints& map);

	// Returns other leading vehicles in the lane which is defined by SDC vehicle state [s, s_d, s_dd, d, d_d, d_dd].
	TOtherCarsTrajectory GetLeadingCarsTrajectoryInLane(const Eigen::VectorXd& sdcStateV6, int nLane, double timeDuration, double timeStep) const;

	// Returns all vehicles in the lane which are close to SDC car's S-position: [sdcStateV6(0)-deltaS, sdcStateV6(0)+deltaS].
	TOtherCarsTrajectory GetOtherCarsTrajectoryInLane(const Eigen::VectorXd& sdcStateV6, int nLane, double timeDuration, double timeStep) const;

	// Returns other leading vehicles in the lane which is defined by SDC vehicle state [s, s_d, s_dd, d, d_d, d_dd].
	TOtherVehicles GetLeadingCarsInLane (const Eigen::VectorXd& sdcStateV6, int nLane, bool bOnlyNearest = false) const;
	// Returns all vehicles in the lane which are close to SDC car's S-position: [sdcStateV6(0)-deltaS, sdcStateV6(0)+deltaS].
	TOtherVehicles GetNearestCarsInLane(const Eigen::VectorXd& sdcStateV6, int nLane, double deltaS = 200) const;

private:
	std::unordered_map<int, bool> m_mapTrackedCars;
	std::unordered_map<int, OtherVehicle> m_mapVehicles;
};
