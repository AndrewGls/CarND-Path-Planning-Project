# include "HighwayMap.h"
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iostream>

using namespace std;

using Eigen::Vector2d;

HighwayMap::HighwayMap()
{
	// Waypoint map to read from
	const string map_file = "../data/highway_map.csv";

	ifstream in_map(map_file.c_str(), ifstream::in);

	Waypoint wp;
	string line;
	while (getline(in_map, line)) {
		istringstream iss(line);
		iss >> wp.x;
		iss >> wp.y;
		iss >> wp.s;
		iss >> wp.d_x;
		iss >> wp.d_y;
		waypoints_.push_back(wp);
	}

	fit_spline();
}


Point HighwayMap::getXYInterpolated(double s, double d) const
{
	s = norm_s(s);

	Point pt(x_spline_(s), y_spline_(s));
	// normal vector
	Point nv(-y_spline_.deriv(1,s), x_spline_.deriv(1,s));
	return pt + nv * d;
}


Vector2d HighwayMap::CalcFrenet(const Point& ptXY, double s_start) const
{
	// Gradient descent is used to find the point (s,d) on the spline, which is closest to point ptXY.
	const double eps = 1.0e-6;
	const double gamma = 0.001;
	const double precision = 1e-12;
	double s = s_start;
	double prev_step_size = s;

	while (prev_step_size > precision)
	{
		const auto prev_s = s;
		s -= gamma * error_deriv(ptXY, prev_s);
		prev_step_size = std::abs(s - prev_s);
	}

	Vector2d p(2);
	p << ptXY.x, ptXY.y;

	const Vector2d p_spline(x_spline_(s), y_spline_(s));
//	std::cout << ptXY.x << ", " << ptXY.y << "| " << p_spline << std::endl;

	const Vector2d p_delta = (p - p_spline).array() / get_normal_at(s).array();
	//std::cout << "p_delta: " << p_delta << std::endl;
	//std::cout << "normal: " << get_normal_at(s) << std::endl;
	const double d = 0.5 * (p_delta(0) + p_delta(1));
	return Vector2d(s, d);
}

double HighwayMap::error_deriv(const Point& pt, double s) const
{
	return -2. * (pt.x - x_spline_(s)) * x_spline_.deriv(1, s)
		- 2. * (pt.y - y_spline_(s)) * y_spline_.deriv(1, s);
}

Vector2d HighwayMap::get_normal_at(double s) const
{
	return Vector2d(-y_spline_.deriv(1, s), x_spline_.deriv(1, s));
}

void HighwayMap::fit_spline()
{
	// Spline fitting
//	sort(waypoints_.begin(), waypoints_.end(), [](const Waypoint& l, const Waypoint& r) {return l.s < r.s; });

	//------------------------------------
	// Fix of kink at the end of the track!
	Waypoint wpF = waypoints_.front();
	Waypoint wpL = waypoints_.back();

	Waypoint wp1 = wpL;
	wp1.s -= max_s_;

	Waypoint wp2 = wpF;
	wp2.s += max_s_;

	waypoints_.insert(waypoints_.begin(), wp1);
	waypoints_.push_back(wp2);
	//------------------------------------


	vector<double> x, y, s;
	x.resize(waypoints_.size());
	y.resize(waypoints_.size());
	s.resize(waypoints_.size());
	for (size_t i = 0; i < waypoints_.size(); i++) {
		const auto& w = waypoints_[i];
		x[i] = w.x;
		y[i] = w.y;
		s[i] = w.s;
	}

	x_spline_.set_points(s, x);
	y_spline_.set_points(s, y);
}

