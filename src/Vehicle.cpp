#include "Vehicle.h"
#include "Waypoint.h"
#include "HighwayMap.h"
#include <iostream>


using namespace std;

namespace {
	static constexpr double delta_t = 20. / 1000.; // 20ms between waypoints
}


//////////////////////////////////////////////////////////////////////////////////////////
Vehicle::Vehicle(const HighwayMap& map)
	: map_(map)
	, state_(fs_keep_lane)
	, set_init_vs_(true)
	, last_s_(0)
	, last_v_(0)
	, last_a_(0)
{
	target_speed_ = speed_limit_;
	n_waypoints_ = static_cast<int>(2. / delta_t + 0.5); // 1000 ms / 20ms
}

//////////////////////////////////////////////////////////////////////////////////////////
void Vehicle::getTrajectory(const CarLocalizationData& newState,
							const vector<double>& previous_path_x,
							const vector<double>& previous_path_y)
{
	updateState(newState);

	if (state_ == fs_keep_lane) {
		trajectory_KeepLine(previous_path_x, previous_path_y);
	}
	else {
		// not implemented yet
	}
}

//////////////////////////////////////////////////////////////////////////////////////////
void Vehicle::updateState(const CarLocalizationData& newState)
{
	vs_ = newState;

	if (set_init_vs_) {
		set_init_vs_ = false;
		last_s_ = newState.s;
		last_v_ = newState.speed *  0.44704;  // convert MPH to m/sec !!!!
		last_a_ = max_a_;
		init_vs_ = newState;
	}
}

//////////////////////////////////////////////////////////////////////////////////////////
void Vehicle::trajectory_KeepLine(const vector<double>& previous_path_x, const vector<double>& previous_path_y)
{
	next_x_vals_.clear();
	next_y_vals_.clear();

	size_t path_size = previous_path_x.size();

	for (size_t i = 0; i < path_size; i++)
	{
		next_x_vals_.push_back(previous_path_x[i]);
		next_y_vals_.push_back(previous_path_y[i]);
	}

	double s = 0;
	double v = 0;
	double d = init_vs_.d;

	const double speed_limit = std::min(target_speed_, speed_limit_);

	if (last_v_ < speed_limit)
		last_a_ = std::min(max_a_, speed_limit - last_v_);
	else
		last_a_ = std::max(-max_a_, speed_limit - last_v_);

	const int n_waypoints = n_waypoints_ - (int)path_size;

	for (int i = n_waypoints; i-- >= 0;)
	{
		s = last_s_ + last_v_ * delta_t + last_a_ * delta_t*delta_t / 2.;
		v = last_v_  + last_a_ * delta_t;

		if (v < speed_limit)
			last_a_ = std::min(max_a_, speed_limit - v);
		else
			last_a_ = std::max(-max_a_, speed_limit - v);

		last_s_ = s;
		last_v_ = v;

		Point pt = map_.getXY(s, -d);
		next_x_vals_.push_back(pt.x);
		next_y_vals_.push_back(pt.y);

		cout.setf(std::ios::fixed);
		cout.precision(2);
		cout << "s: " << s << " d: " << d << " (" << pt.x << ", " << pt.y << ")" << std::endl;
	}
}
