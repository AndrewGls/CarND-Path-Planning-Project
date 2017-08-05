#pragma once

#include "Types.h"
#include "Behavior.h"
#include <vector>
#include <memory>

class HighwayMap;
class Vehicle;


class Vehicle
{
public:
	Vehicle(const HighwayMap& map);

	std::vector<SensorFusionData>& getSensorFusionStorage() { return sf_data_; }
	void updateTrajectory(const CarLocalizationData& newState, const std::vector<double>& previous_path_x, const std::vector<double>& previous_path_y);

	double speed_limit() const { return speed_limit_; }
	double target_speed() const { return target_speed_; }
	double max_acceleration() const { return max_a_; }

	const HighwayMap& get_map() const { return map_; }
	const std::vector<SensorFusionData>& sensor_fucsion() const { return sf_data_; }

	double braking_distance(double speed) const;
	double get_min_distance(double speed) const;

	std::vector<double>& get_next_x_vals() { return next_x_vals_; }
	std::vector<double>& get_next_y_vals() { return next_y_vals_; }

private:

private:
	const HighwayMap& map_;
	std::vector<SensorFusionData> sf_data_;
	FiniteState state_;
	bool initDone_;

	// Active prediction path size is (predictionHorizont_ - reactiveHorizont_)
	double predictionHorizont_ = 2; // in secs
	double reactiveHorizont_ = 1;   // in secs

	//---------------
	// Current stale
	double currTime_ = 0; // current time
	Eigen::VectorXd currStateV6_; // [s, v, a, d, d_d, d_dd]
	//---------------

	Behavior behavior_;

	const double speed_limit_ = (50. - 2.) * 0.44704; // speed limit in m/s
	double target_speed_;			// m/s
	const double max_a_ = 4; // max car acceleration
	const double mu_ = 0.7;  // coefficient of friction between the road surface and the tires for dry road

	std::vector<double> next_x_vals_;
	std::vector<double> next_y_vals_;
};


