#pragma once

#include <vector>
#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>
#include "Eigen-3.3/Eigen/Dense"


using Eigen::MatrixXd;
using Eigen::VectorXd;


namespace Utils
{
	// For converting back and forth between radians and degrees.
	constexpr double pi() { return M_PI; }
	double deg2rad(double x) { return x * pi() / 180; }
	double rad2deg(double x) { return x * 180 / pi(); }


	double distance(double x1, double y1, double x2, double y2)
	{
		return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
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


	/************************************************************************************************
		Calculate the Jerk Minimizing Trajectory that connects the initial state to the final state in time T.
		INPUTS

		start - the vehicles start location given as a length three array
		corresponding to initial values of[s, s_dot, s_double_dot]

		end - the desired end state for vehicle.Like "start" this is a
		length three array.

		T - The duration, in seconds, over which this maneuver should occur.

		OUTPUT
		an array of length 6, each value corresponding to a coefficent in the polynomial
		s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

		EXAMPLE

		> JMT([0, 10, 0], [10, 10, 0], 1)
		[0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    ************************************************************************************************/
	inline std::vector<double> JMT(double T, const std::vector< double>& start, const std::vector <double>& end, std::vector<double>& result)
	{
		MatrixXd A = MatrixXd(3, 3);
		A << T*T*T, T*T*T*T, T*T*T*T*T,
			3 * T*T, 4 * T*T*T, 5 * T*T*T*T,
			6 * T, 12 * T*T, 20 * T*T*T;

		MatrixXd B = MatrixXd(3, 1);
		B << end[0] - (start[0] + start[1] * T + .5*start[2] * T*T),
			end[1] - (start[1] + start[2] * T),
			end[2] - start[2];

		MatrixXd Ai = A.inverse();
		MatrixXd C = Ai*B;

		result = { start[0], start[1], .5*start[2] };
		for (int i = 0; i < C.size(); i++)
		{
			result.push_back(C.data()[i]);
		}

		return result;

	}

}
