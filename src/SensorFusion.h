#pragma once

#include "Types.h"
#include "HighwayMap.h"
#include "OtherVehicle.h"
#include <unordered_map>

using TOtherVehicles = std::vector<OtherVehicle>;


// Helper class to predict position of vehicles after delayTime.
class SensorFusion
{
public:
	SensorFusion() {}

	void predict(double deltaTime);
	void update (const std::vector<SensorFusionData>& sensorFusion, double currS, const HighwayMap& map);

	// Returns other leading vehicles in the lane which is defined by SDC vehicle state [s, s_d, s_dd, d, d_d, d_dd].
	TOtherVehicles getLeadingVehiclesInLane (const Eigen::VectorXd& sdcStateV6) const;
	// Returns all vehicles in the lane which are close to SDC car's S-position: [sdcStateV6(0)-deltaS, sdcStateV6(0)+deltaS].
	TOtherVehicles getNearestVehiclesInLane(const Eigen::VectorXd& sdcStateV6, double deltaS = 200) const;

private:
	std::unordered_map<int, bool> m_mapUpdate;
	std::unordered_map<int, OtherVehicle> m_mapVehicles;
};
