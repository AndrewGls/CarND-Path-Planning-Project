#pragma once

#include <vector>
#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>
#include <array>
#include "Eigen-3.3/Eigen/Dense"


using Eigen::MatrixXd;
using Eigen::VectorXd;


namespace Utils
{
	constexpr double MaxDoubleVal = 1.e10;
	constexpr double MinDoubleVal = -1.e10;
	constexpr double delta_t = 0.02; // 20ms between waypoints
	constexpr double LaneWidth = 4;

	// For converting back and forth between radians and degrees.
	constexpr double pi() { return M_PI; }
	inline double deg2rad(double x) { return x * pi() / 180; }
	inline double rad2deg(double x) { return x * 180 / pi(); }


	inline double distance(double x1, double y1, double x2, double y2)
	{
		return std::sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
	}

	inline double distance(double x1, double x2)
	{
		return std::abs(x2 - x1);
	}

	// Returns lane number -1, 0, 1, 2 for specified d coord. in Frenet frame.
	inline int DtoLaneNumber(double d)
	{
		if (d > 0 || d < -12)
			return -1; // outside of right-driving road.
		if (d < -8)
			return 0;
		else if (d < -4)
			return 1;
		return 2;
	}

	// Returns D - center of Lane for specified Lane number
	inline double LaneNumberToD(int nLaneNumber)
	{
		constexpr std::array<double, 3> laneCenter = { -9.75, -6.0, -2.0 };
		assert(nLaneNumber >= 0 && nLaneNumber < 3);
		return laneCenter[nLaneNumber];
	}
}
