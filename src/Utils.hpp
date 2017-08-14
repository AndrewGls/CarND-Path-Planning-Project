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


	inline double braking_distance(double speed, double mu = 0.7)
	{
		static constexpr double g = 9.8; // g is the gravity of Earth
		return speed*speed / (2 * mu * g);
	}


	int ClosestWaypoint(double x, double y, std::vector<double> maps_x, std::vector<double> maps_y);
	int NextWaypoint(double x, double y, double theta, std::vector<double> maps_x, std::vector<double> maps_y);
	// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
	std::vector<double> getFrenet(double x, double y, double theta, std::vector<double> maps_x, std::vector<double> maps_y);
	// Transform from Frenet s,d coordinates to Cartesian x,y
	std::vector<double> getXY(double s, double d, std::vector<double> maps_s, std::vector<double> maps_x, std::vector<double> maps_y);


	inline int ClosestWaypoint(double x, double y, std::vector<double> maps_x, std::vector<double> maps_y)
	{

		double closestLen = 100000; //large number
		int closestWaypoint = 0;

		for (int i = 0; i < maps_x.size(); i++)
		{
			double map_x = maps_x[i];
			double map_y = maps_y[i];
			double dist = distance(x, y, map_x, map_y);
			if (dist < closestLen)
			{
				closestLen = dist;
				closestWaypoint = i;
			}

		}

		return closestWaypoint;

	}


	inline int NextWaypoint(double x, double y, double theta, std::vector<double> maps_x, std::vector<double> maps_y)
	{

		int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

		double map_x = maps_x[closestWaypoint];
		double map_y = maps_y[closestWaypoint];

		double heading = atan2((map_y - y), (map_x - x));

		double angle = abs(theta - heading);

		if (angle > pi() / 4)
		{
			closestWaypoint++;
		}

		return closestWaypoint;

	}

	// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
	inline std::vector<double> getFrenet(double x, double y, double theta, std::vector<double> maps_x, std::vector<double> maps_y)
	{
		int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

		int prev_wp;
		prev_wp = next_wp - 1;
		if (next_wp == 0)
		{
			prev_wp = maps_x.size() - 1;
		}

		double n_x = maps_x[next_wp] - maps_x[prev_wp];
		double n_y = maps_y[next_wp] - maps_y[prev_wp];
		double x_x = x - maps_x[prev_wp];
		double x_y = y - maps_y[prev_wp];

		// find the projection of x onto n
		double proj_norm = (x_x*n_x + x_y*n_y) / (n_x*n_x + n_y*n_y);
		double proj_x = proj_norm*n_x;
		double proj_y = proj_norm*n_y;

		double frenet_d = distance(x_x, x_y, proj_x, proj_y);

		//see if d value is positive or negative by comparing it to a center point

		double center_x = 1000 - maps_x[prev_wp];
		double center_y = 2000 - maps_y[prev_wp];
		double centerToPos = distance(center_x, center_y, x_x, x_y);
		double centerToRef = distance(center_x, center_y, proj_x, proj_y);

		if (centerToPos <= centerToRef)
		{
			frenet_d *= -1;
		}

		// calculate s value
		double frenet_s = 0;
		for (int i = 0; i < prev_wp; i++)
		{
			frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
		}

		frenet_s += distance(0, 0, proj_x, proj_y);

		return { frenet_s,frenet_d };

	}


	// Transform from Frenet s,d coordinates to Cartesian x,y
	inline std::vector<double> getXY(double s, double d, std::vector<double> maps_s, std::vector<double> maps_x, std::vector<double> maps_y)
	{
		int prev_wp = -1;

		while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1)))
		{
			prev_wp++;
		}

		int wp2 = (prev_wp + 1) % maps_x.size();

		double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
		// the x,y,s along the segment
		double seg_s = (s - maps_s[prev_wp]);

		double seg_x = maps_x[prev_wp] + seg_s*cos(heading);
		double seg_y = maps_y[prev_wp] + seg_s*sin(heading);

		double perp_heading = heading - pi() / 2;

		double x = seg_x + d*cos(perp_heading);
		double y = seg_y + d*sin(perp_heading);

		return { x,y };

	}



}
