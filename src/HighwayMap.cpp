# include "HighwayMap.h"
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>

using namespace std;

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


Point HighwayMap::getXY(double s, double d) const
{
	s = norm_s(s);

	Point pt(x_spline_(s), y_spline_(s));

	// add d to final point!

	return pt;
}


void HighwayMap::fit_spline()
{
	// Spline fitting
	sort(waypoints_.begin(), waypoints_.end(), [](const Waypoint& l, const Waypoint& r) {return l.s < r.s; });

	vector<double> x, y, s, d_x, d_y;
	x.resize(waypoints_.size());
	y.resize(waypoints_.size());
	s.resize(waypoints_.size());
	d_x.resize(waypoints_.size());
	d_y.resize(waypoints_.size());
	for (size_t i = 0; i < waypoints_.size(); i++) {
		const auto& w = waypoints_[i];
		x[i] = w.x;
		y[i] = w.y;
		s[i] = w.s;
		d_x[i] = w.d_x;
		d_y[i] = w.d_y;
	}

	x_spline_.set_points(s, x);
	y_spline_.set_points(s, y);
	dx_spline_.set_points(s, d_x);
	dy_spline_.set_points(s, d_y);
}

