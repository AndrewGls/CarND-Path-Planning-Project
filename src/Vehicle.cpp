#include "Vehicle.h"
#include "Waypoint.h"
#include "HighwayMap.h"
#include <iostream>


using namespace std;


//////////////////////////////////////////////////////////////////////////////////////////
Vehicle::Vehicle(const HighwayMap& map)
	: map_(map)
	, state_(fs_keep_lane)
	, n_waypoints_(50)
	, set_init_vs_(true)
	, last_s_(0)
	, s_dot_(0.4)
{

}

//////////////////////////////////////////////////////////////////////////////////////////
void Vehicle::getTrajectory(const VehicleState& newState,
							const vector<double>& previous_path_x,
							const vector<double>& previous_path_y)
{
	updateState(newState);

	next_x_vals_.clear();
	next_y_vals_.clear();

	int path_size = previous_path_x.size();

	for (int i = 0; i < path_size; i++)
	{
		next_x_vals_.push_back(previous_path_x[i]);
		next_y_vals_.push_back(previous_path_y[i]);
	}

	if (state_ == fs_keep_lane) {
		trajectory_KeepLine(previous_path_x, previous_path_y);
	}
	else {
		// not implemented yet
	}
}

//////////////////////////////////////////////////////////////////////////////////////////
void Vehicle::updateState(const VehicleState& newState)
{
	vs_ = newState;

	if (set_init_vs_) {
		set_init_vs_ = false;
		init_vs_ = newState;
	}
}

//////////////////////////////////////////////////////////////////////////////////////////
void Vehicle::trajectory_KeepLine(const vector<double>& previous_path_x,
								  const vector<double>& previous_path_y)
{
	int path_size = previous_path_x.size();

	double s = vs_.s;
	double d = init_vs_.d;

	if (path_size)
	{
		s = last_s_;
	}

	for (int i = n_waypoints_ - path_size; i-- > 0;)
	{
		s += s_dot_;
		Point pt = map_.getXY(s, -d);
		last_s_ = s;
		cout << "s: " << s << " d: " << d << " ptx: " << pt.x << " pty: " << pt.y << std::endl;
		next_x_vals_.push_back(pt.x);
		next_y_vals_.push_back(pt.y);
	}

}
