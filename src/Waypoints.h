#pragma once

#include "Types.h"
#include "spline.h"
#include <vector>
#include "Eigen-3.3/Eigen/Core"

class Waypoints
{
public:
	Waypoints();

	// Transform from Frenet s,d coordinates to Cartesian x,y using bspline for interpolation.
	Point GetXYInterpolated(double s, double d) const;

	// Returns point [s, d] on the spline (Highway-Map) which is the closest point to the (x,y).
	// s_start defines the initial position on the spline.
	Eigen::Vector2d CalcFrenet(const Point& ptXY, double s_start) const;

	double max_s() const { return m_max_S; }

private:
	void FitSpline();
	double NormalizeS(double s) const;
	// Calculates derivative d/ds of error function [(x - x_spline)^2 + (y - y_spline)^2].
	double ErrorDeriv(const Point& pt, double s) const;
	// Returns normal to the spline at point s.
	Eigen::Vector2d GetNormalAt(double s) const;

private:
	// The max s value before wrapping around the track back to 0
	const double m_max_S = 6945.554;

	std::vector<Waypoint> m_waypoints;
	tk::spline m_x_spline; // x(s)
	tk::spline m_y_spline; // y(s)
};


inline double Waypoints::NormalizeS(double s) const
{
	return (s > m_max_S) ? (s -= m_max_S) : (s < -m_max_S ? (s += m_max_S) : s);
}
