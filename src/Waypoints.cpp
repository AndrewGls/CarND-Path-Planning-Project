#include "Waypoints.h"
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iostream>

using namespace std;

using Eigen::Vector2d;

Waypoints::Waypoints()
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
		m_waypoints.push_back(wp);
	}

	FitSpline();
}


Point Waypoints::GetXYInterpolated(double s, double d) const
{
	s = NormalizeS(s);

	Point pt(m_x_spline(s), m_y_spline(s));
	// normal vector
	Point nv(-m_y_spline.deriv(1,s), m_x_spline.deriv(1,s));
	return pt + nv * d;
}


Vector2d Waypoints::CalcFrenet(const Point& ptXY, double s_start) const
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
		s -= gamma * ErrorDeriv(ptXY, prev_s);
		prev_step_size = std::abs(s - prev_s);
	}

	Vector2d p(2);
	p << ptXY.x, ptXY.y;

	const Vector2d p_spline(m_x_spline(s), m_y_spline(s));
//	std::cout << ptXY.x << ", " << ptXY.y << "| " << p_spline << std::endl;

	const Vector2d p_delta = (p - p_spline).array() / GetNormalAt(s).array();
	//std::cout << "p_delta: " << p_delta << std::endl;
	//std::cout << "normal: " << GetNormalAt(s) << std::endl;
	const double d = 0.5 * (p_delta(0) + p_delta(1));
	return Vector2d(s, d);
}

double Waypoints::ErrorDeriv(const Point& pt, double s) const
{
	return -2. * (pt.x - m_x_spline(s)) * m_x_spline.deriv(1, s)
		- 2. * (pt.y - m_y_spline(s)) * m_y_spline.deriv(1, s);
}

Vector2d Waypoints::GetNormalAt(double s) const
{
	return Vector2d(-m_y_spline.deriv(1, s), m_x_spline.deriv(1, s));
}

void Waypoints::FitSpline()
{
	// Spline fitting
//	sort(m_waypoints.begin(), m_waypoints.end(), [](const Waypoint& l, const Waypoint& r) {return l.s < r.s; });

#if 1
	//------------------------------------
	// Fix of kink at the end of the track!
	Waypoint wpF = m_waypoints.front();
	Waypoint wpL = m_waypoints.back();

	Waypoint wp1 = wpL;
	wp1.s -= m_max_S;

	Waypoint wp2 = wpF;
	wp2.s += m_max_S;

	m_waypoints.insert(m_waypoints.begin(), wp1);
	m_waypoints.push_back(wp2);
	//------------------------------------
#endif


	vector<double> x, y, s;
	x.resize(m_waypoints.size());
	y.resize(m_waypoints.size());
	s.resize(m_waypoints.size());
	for (size_t i = 0; i < m_waypoints.size(); i++) {
		const auto& w = m_waypoints[i];
		x[i] = w.x;
		y[i] = w.y;
		s[i] = w.s;
	}

	m_x_spline.set_points(s, x);
	m_y_spline.set_points(s, y);
}

