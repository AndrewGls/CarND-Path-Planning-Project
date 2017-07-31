#pragma once

#include <vector>

class HighwayMap;


struct VehicleState
{
	double x;
	double y;
	double s;
	double d;
	double yaw;
	double speed;

	VehicleState(double x = 0, double y = 0, double s = 0, double d = 0, double yaw = 0, double speed = 0)
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

	void getTrajectory(const VehicleState& newState, const std::vector<double>& previous_path_x, const std::vector<double>& previous_path_y);

	std::vector<double>& get_next_x_vals() { return next_x_vals_; }
	std::vector<double>& get_next_y_vals() { return next_y_vals_; }

private:
	void updateState(const VehicleState& newState);
	void trajectory_KeepLine(const std::vector<double>& previous_path_x, const std::vector<double>& previous_path_y);

private:
	const HighwayMap& map_;
	FiniteState state_;
	int n_waypoints_; // number of waypoints in predicted trajectory

	bool set_init_vs_;
	VehicleState init_vs_;	// initial state of vehicle
	VehicleState vs_;		// current state of vehicle
	double last_s_;

	double s_dot_;

	std::vector<double> next_x_vals_;
	std::vector<double> next_y_vals_;
};