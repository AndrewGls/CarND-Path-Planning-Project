#pragma once

#include "Types.h"
#include "HighwayMap.h"
#include <vector>


class SensorFusion
{
public:
	// delay_time is used to correct prosition of vehicles for currecmt time.
	SensorFusion(const std::vector<SensorFusionData>& sensorFusion, double delay_time, const HighwayMap& map);
};
