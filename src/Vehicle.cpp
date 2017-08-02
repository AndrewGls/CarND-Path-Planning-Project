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

	tc_keep_line_ = std::make_unique<TrajectoryController>();
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
#if 0
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
#else
void Vehicle::trajectory_KeepLine(const vector<double>& previous_path_x, const vector<double>& previous_path_y)
{
	tc_keep_line_->PredictTrajectory(*this, target_speed_, last_s_, last_v_, last_a_, init_vs_.d, previous_path_x, previous_path_y);
	tc_keep_line_->loadTrajectory(*this);
}
#endif


//////////////////////////////////////////////////////////////////////////////////////////
TrajectoryController::TrajectoryController()
	: last_s_(0)
	, last_v_(0)
	, last_a_(0)
	, last_d_(0)
{
	n_waypoints_ = static_cast<int>(2. / delta_t + 0.5); // 1000 ms / 20ms
}


void TrajectoryController::PredictTrajectory (const Vehicle& vehicle,
											  double target_speed,
											  double init_s,
											  double init_v,
											  double init_a,
											  double init_d,
											  const vector<double>& previous_path_x,
											  const vector<double>& previous_path_y)
{
	auto max_a = vehicle.max_acceleration();

	size_t path_size = previous_path_x.size();
	const int n_waypoints = n_waypoints_ - (int)path_size;
	const HighwayMap& map = vehicle.get_map();

	next_x_vals_.clear();
	next_y_vals_.clear();

	{
		std::vector<double> next_s_vals_tmp;
		std::vector<double> next_d_vals_tmp;

		std::swap(next_s_vals_, next_s_vals_tmp);
		std::swap(next_d_vals_, next_d_vals_tmp);

		for (size_t i = 0; i < path_size; i++)
		{
			next_x_vals_.push_back(previous_path_x[i]);
			next_y_vals_.push_back(previous_path_y[i]);

			next_s_vals_.push_back(next_s_vals_tmp[i]);
			next_d_vals_.push_back(next_d_vals_tmp[i]);
		}
	}


	double s = 0;
	double v = 0;
	double d = init_d;

	if (!path_size) {
		last_s_ = init_s;
	}

	const double speed_limit = std::min(target_speed, vehicle.speed_limit());

	if (last_v_ < speed_limit)
		last_a_ = std::min(max_a, speed_limit - last_v_);
	else
		last_a_ = std::max(-max_a, speed_limit - last_v_);

	for (int i = n_waypoints; i-- >= 0;)
	{
		s = last_s_ + last_v_ * delta_t + last_a_ * delta_t*delta_t / 2.;
		v = last_v_ + last_a_ * delta_t;

		if (v < speed_limit)
			last_a_ = std::min(max_a, speed_limit - v);
		else
			last_a_ = std::max(-max_a, speed_limit - v);

		last_s_ = s;
		last_v_ = v;

		next_s_vals_.push_back(s);
		next_d_vals_.push_back(d);

		Point pt = map.getXY(s, -d);

		next_x_vals_.push_back(pt.x);
		next_y_vals_.push_back(pt.y);

		cout.setf(std::ios::fixed);
		cout.precision(2);

		cout << "s: " << s << " d: " << d << " (" << pt.x << ", " << pt.y << ")" << std::endl;
	}
}


void TrajectoryController::loadTrajectory(Vehicle& vehicle)
{
	vehicle.get_next_x_vals() = next_x_vals_;
	vehicle.get_next_y_vals() = next_y_vals_;

	vehicle.set_last_s(last_s_);
	vehicle.set_last_v(last_v_);
	vehicle.set_last_a(last_a_);
//	vehicle.set_last_d(last_d_);
}
