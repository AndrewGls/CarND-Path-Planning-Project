#pragma once

#include "Waypoint.h"
#include "spline.h"
#include <vector>


class HighwayMap
{
public:
	HighwayMap();

	// Transform from Frenet s,d coordinates to Cartesian x,y
	Point getXY(double s, double d) const;

private:
	void fit_spline();
	double norm_s(double s) const;

private:
	// The max s value before wrapping around the track back to 0
	const double max_s_ = 6945.554;

	std::vector<Waypoint> waypoints_;
	tk::spline x_spline_; // x(s)
	tk::spline y_spline_; // y(s)
//	tk::spline dx_spline_; // d_x(s)
//	tk::spline dy_spline_; // d_y(s)
};


inline double HighwayMap::norm_s(double s) const
{
	return (s > max_s_) ? (s -= max_s_) : (s < -max_s_ ? (s += max_s_) : s);
}
