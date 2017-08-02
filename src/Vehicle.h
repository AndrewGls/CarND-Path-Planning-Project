#pragma once

#include "Types.h"
#include <vector>
#include <memory>

class HighwayMap;
class Vehicle;


class TrajectoryController
{
public:
	TrajectoryController();

	void PredictTrajectory(
		const Vehicle& vehicle,
		double target_speed,
		double init_s,
		double init_v,
		double init_a,
		double init_d,
		const std::vector<double>& previous_path_x,
		const std::vector<double>& previous_path_y);

	void loadTrajectory(Vehicle& vehicle);

private:
	int n_waypoints_; // number of waypoints in predicted trajectory

//	double target_speed_;

	std::vector<double> next_x_vals_;
	std::vector<double> next_y_vals_;

	std::vector<double> next_s_vals_;
	std::vector<double> next_d_vals_;

	std::vector<double> next_v_vals_;
	std::vector<double> next_a_vals_;
};



class Vehicle
{
public:
	Vehicle(const HighwayMap& map);

	std::vector<SensorFusionData>& getSensorFusionStorage() { return sf_data_; }
	void getTrajectory(const CarLocalizationData& newState, const std::vector<double>& previous_path_x, const std::vector<double>& previous_path_y);

	double speed_limit() const { return speed_limit_; }
	double target_speed() const { return target_speed_; }
	double max_acceleration() const { return max_a_; }

	const HighwayMap& get_map() const { return map_; }
	const std::vector<SensorFusionData>& sensor_fucsion() const { return sf_data_; }

	double braking_distance(double speed) const;
	double get_max_distance(double speed) const;

	std::vector<double>& get_next_x_vals() { return next_x_vals_; }
	std::vector<double>& get_next_y_vals() { return next_y_vals_; }

	double last_s() const { return last_s_; }
	double last_v() const { return last_v_; }
	double last_a() const { return last_a_; }
//	double last_d() const { return last_d_; }

	void set_last_s(double v) { last_s_ = v; }
	void set_last_v(double v) { last_v_ = v; }
	void set_last_a(double v) { last_a_ = v; }
//	void last_d() { return last_d_; }

private:
	void updateState(const CarLocalizationData& newState);
	void trajectory_KeepLine(const std::vector<double>& previous_path_x, const std::vector<double>& previous_path_y);

private:
	const HighwayMap& map_;
	std::vector<SensorFusionData> sf_data_;
	FiniteState state_;
	int n_waypoints_; // number of waypoints in predicted trajectory

	bool set_init_vs_;
	CarLocalizationData init_vs_;	// initial state of vehicle
	CarLocalizationData vs_;		// current state of vehicle
	double last_s_;					// the last waypoint in trajectory, calculated during previous call
	double last_v_;					// m/sec
	double last_a_;					// m/sec^2
	const double speed_limit_ = (50. - 2.) * 0.44704; // speed limit in m/s
	double target_speed_;			// m/s
	const double max_a_ = 4; // max car acceleration
	const double mu_ = 0.7;  // coefficient of friction between the road surface and the tires for dry road

	std::vector<double> next_x_vals_;
	std::vector<double> next_y_vals_;

	std::unique_ptr<TrajectoryController> tc_keep_line_;
};


