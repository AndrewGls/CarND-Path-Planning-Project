#pragma once

#include "Waypoint.h"
#include "spline.h"
#include <vector>
#include "Eigen-3.3/Eigen/Core"

class HighwayMap
{
public:
	HighwayMap();

	// Transform from Frenet s,d coordinates to Cartesian x,y using bspline for interpolation.
	Point getXYInterpolated(double s, double d) const;

	// Returns point [s, d] on the spline (Highway-Map) which is the closest point to the (x,y).
	// s_start defines the initial position on the spline.
	Eigen::Vector2d CalcFrenet(const Point& ptXY, double s_start) const;

private:
	void fit_spline();
	double norm_s(double s) const;
	// Calculates derivative d/ds of error function [(x - x_spline)^2 + (y - y_spline)^2].
	double error_deriv(const Point& pt, double s) const;
	// Returns normal to the spline at point s.
	Eigen::Vector2d get_normal_at(double s) const;

private:
	// The max s value before wrapping around the track back to 0
	const double max_s_ = 6945.554;

	std::vector<Waypoint> waypoints_;
	tk::spline x_spline_; // x(s)
	tk::spline y_spline_; // y(s)
};


inline double HighwayMap::norm_s(double s) const
{
	return (s > max_s_) ? (s -= max_s_) : (s < -max_s_ ? (s += max_s_) : s);
}
