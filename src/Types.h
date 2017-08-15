#pragma once

#ifndef TYPES_H
#define TYPES_H

#include "Eigen-3.3/Eigen/Eigen"
#include "Eigen-3.3/Eigen/Core"
#include <limits>

//#define VERBOSE_NEXT_XY
//#define VERBOSE_BEST_TRAJECTORY
//#define VERBOSE_TRAJECTORIES
//#define VERBOSE_OTHER_CARS_CHECK_TRANSFORM
//#define VERBOSE_OTHER_LEADING_CARS
//#define VERBOSE_OTHER_IGNORED_CARS

//#define VERBOSE_STATE


struct Point
{
	double x;
	double y;

	Point(double x = 0, double y = 0) : x(x), y(y) {}

	Point& operator*(double n) { x *= n; y *= n; return *this; }
	Point& operator+(const Point& r) {
		if (this != &r) {
			x += r.x;
			y += r.y;
		}
		return *this;
	}
};


struct Waypoint
{
	double x;
	double y;
	double s;
	double d_x;
	double d_y;

	Waypoint(double x = 0, double y = 0, double s = 0, double d_x = 0, double d_y = 0)
		: x(x), y(y), s(s), d_x(d_x), d_y(d_y) {}
};


struct CarLocalizationData
{
	double x;
	double y;
	double s;
	double d;
	double yaw;
	double speed;

	CarLocalizationData(double x = 0, double y = 0, double s = 0, double d = 0, double yaw = 0, double speed = 0)
		: x(x), y(y), s(s), d(d), yaw(yaw), speed(speed) {}
};


struct SensorFusionData
{
	int id;
	double x;
	double y;
	double vx;
	double vy;
	double s;
	double d;
};

#endif // TYPES_H
