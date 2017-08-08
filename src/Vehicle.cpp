#include "Vehicle.h"
#include "Waypoint.h"
#include "HighwayMap.h"
#include "SensorFusion.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Vector2d;


using namespace std;

namespace {
//	static constexpr double delta_t = 20. / 1000.; // 20ms between waypoints
	static constexpr double buffer_distance = 2;   // m units, distance between vehicles when speed is clise to 0.
	static constexpr double lane_wide = 4;        // each lane is 4 m wide
}


//////////////////////////////////////////////////////////////////////////////////////////
Vehicle::Vehicle(const HighwayMap& map)
	: map_(map)
	, state_(fs_keep_lane)
	, initDone_(false)
{
//	std::cout.setf(std::ios::fixed);
//	std::cout.precision(2);

	currStateV6_ = VectorXd::Zero(6);

	target_speed_ = speed_limit_;
}

//////////////////////////////////////////////////////////////////////////////////////////
void Vehicle::updateTrajectory(const CarLocalizationData& newState,
							   const vector<double>& previous_path_x,
							   const vector<double>& previous_path_y)
{
	const int nPredictionPathSize = int(predictionHorizont_ * 1000) / 20;
	const int nReactivePathSize = int(reactiveHorizont_ * 1000) / 20;

	next_x_vals_.clear();
	next_y_vals_.clear();

	if (!initDone_)
	{
		// first time/reset
		const double velocity = newState.speed * 0.44704;  // convert MPH to m/sec !!!!
		currTime_ = 0;
		const Vector2d& initialFrenet = map_.CalcFrenet(Point(newState.x, newState.y), newState.s);
		currStateV6_ << initialFrenet(0), velocity, 0., initialFrenet(1), 0., 0.;
		initDone_ = true;
	}

	const int nPrevPathSize = (int) previous_path_x.size();
	int nPrevPredictionPathSize = nPrevPathSize - (nPredictionPathSize - nReactivePathSize);
	nPrevPredictionPathSize = max(0, nPrevPredictionPathSize);
	for (int i = 0; i < nPrevPredictionPathSize; ++i)
	{
		next_x_vals_.push_back(previous_path_x[i]);
		next_y_vals_.push_back(previous_path_y[i]);
	}

	// Predict position of vehicles after delay-time:
	const double delayTime = nPrevPredictionPathSize * delta_t;
	sensorFusion_.update(sf_data_, currStateV6_(0), map_);
	sensorFusion_.predict(delayTime);

	TrajectoryPtr pTraj = behavior_.optimalTrajectory(currStateV6_, currTime_, sensorFusion_);

#ifdef VERBOSE_NEXT_XY
	cout << "------- Driving path ----------" << endl;
#endif

	double curr_time = currTime_;
	Eigen::VectorXd curr_state = currStateV6_;

	for (int i = nPrevPredictionPathSize; i < nPredictionPathSize; ++i)
	{
		if (i == nReactivePathSize)
		{
			currTime_ = curr_time;
			currStateV6_ = curr_state;
		}

		const auto pt = map_.getXYInterpolated(curr_state(0), curr_state(3));
		curr_time += delta_t;
		curr_state = pTraj->evalaluateStateAt(curr_time);

		next_x_vals_.push_back(pt.x);
		next_y_vals_.push_back(pt.y);

#ifdef VERBOSE_NEXT_XY
		cout << "s: " << curr_state(0)
			<< " d: " << curr_state(3)
			<< " (" << pt.x << ", " << pt.y << ") "
			<< " v: " << curr_state(1)
			<< " a: " << curr_state(2);
		cout << endl;
#endif // VERBOSE_NEXT_XY
	}

	//assert(next_x_vals_.size() == nPredictionPathSize);
}

//////////////////////////////////////////////////////////////////////////////////////////
double Vehicle::braking_distance(double speed) const
{
	static constexpr double g = 9.8; // g is the gravity of Earth
	return speed*speed / (2 * mu_ * g);
}


//////////////////////////////////////////////////////////////////////////////////////////
double Vehicle::get_min_distance(double speed) const
{
	return braking_distance(speed) + buffer_distance;
}
