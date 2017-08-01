#pragma once

#include <vector>

class HighwayMap;


struct CarLocalizationData
{
	double x;
	double y;
	double s;
	double d;
	double yaw;
	double speed;

	CarLocalizationData(double x = 0, double y = 0, double s = 0, double d = 0, double yaw = 0, double speed = 0)
	: x(x), y(y), s(s), d(d), yaw(yaw), speed(speed) {}
};


enum FiniteState
{
	fs_keep_lane
};


class Vehicle
{
public:
	Vehicle(const HighwayMap& map);

	void getTrajectory(const CarLocalizationData& newState, const std::vector<double>& previous_path_x, const std::vector<double>& previous_path_y);

	std::vector<double>& get_next_x_vals() { return next_x_vals_; }
	std::vector<double>& get_next_y_vals() { return next_y_vals_; }

private:
	void updateState(const CarLocalizationData& newState);
	void trajectory_KeepLine(const std::vector<double>& previous_path_x, const std::vector<double>& previous_path_y);

private:
	const HighwayMap& map_;
	FiniteState state_;
	int n_waypoints_; // number of waypoints in predicted trajectory

	bool set_init_vs_;
	CarLocalizationData init_vs_;	// initial state of vehicle
	CarLocalizationData vs_;		// current state of vehicle
	double last_s_;					// the last waypoint in trajectory, calculated during previous call
	double last_v_;					// m/sec
	double last_a_;					// m/sec^2
	const double max_v_ = (50. - 2.) * 0.44704; // speed limit in m/s
	const double max_a_ = 4; // max car acceleration

	std::vector<double> next_x_vals_;
	std::vector<double> next_y_vals_;
};