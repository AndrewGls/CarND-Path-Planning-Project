#pragma once

#include "Types.h"
#include "HighwayMap.h"
#include "OtherVehicle.h"
#include <vector>

using TOtherVehicles = std::vector<OtherVehicle>;


// Helper class to predict position of vehicles after delayTime.
class SensorFusion
{
public:
	SensorFusion();

	void update (const std::vector<SensorFusionData>& sensorFusion, double delayTime, const HighwayMap& map);

	// Returns other leading vehicles in the line which is defined by SDC vehicle state [s, s_d, s_dd, d, d_d, d_dd].
	TOtherVehicles getLeadingVehiclesInLane (const Eigen::VectorXd& sdcStateV6) const;

private:
	TOtherVehicles m_vehicles;
};
