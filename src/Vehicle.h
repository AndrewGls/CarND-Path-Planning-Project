#pragma once

#include "Types.h"
#include "PathPlanner.h"
#include "SensorFusion.h"
#include <vector>
#include <memory>

class Waypoints;
class Vehicle;


class Vehicle
{
public:
	Vehicle(const Waypoints& map);

	std::vector<SensorFusionData>& SensorFusionStorage() { return m_sensorFusionData; }
	void UpdateTrajectory(const CarLocalizationData& newState, const std::vector<double>& previous_path_x, const std::vector<double>& previous_path_y);

	std::vector<double>& get_next_x_vals() { return m_next_x_vals; }
	std::vector<double>& get_next_y_vals() { return m_next_y_vals; }

private:
	PathPlanner m_behavior;
	SensorFusion m_sensorFusion;
	const Waypoints& m_waypoints;
	std::vector<SensorFusionData> m_sensorFusionData;
	std::vector<double> m_next_x_vals;
	std::vector<double> m_next_y_vals;

	bool m_isInitDone;
	// Current stale
	double m_currTime = 0; // current time
	Eigen::VectorXd m_currStateV6; // [s, v, a, d, d_d, d_dd]

	// Active prediction path size is (m_predictionHorizont - m_reactiveHorizont)
	const double m_predictionHorizont = 2; // in secs
	const double m_reactiveHorizont = 1;   // in secs
};


