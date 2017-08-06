#pragma once

#include "Eigen-3.3/Eigen/Eigen"
#include "Eigen-3.3/Eigen/Core"
#include <limits>

//#define VERBOSE_NEXT_XY
#define VERBOSE_LINE_KEEPING
//#define VERBOSE_OTHER_CARS_CHECK_TRANSFORM
#define VERBOSE_OTHER_LEADING_CARS
#define VERBOSE_OTHER_IGNORED_CARS

static constexpr double MaxDoubleVal = 1.e10;
static constexpr double MinDoubleVal = -1.e10;

static constexpr double delta_t = 20. / 1000.; // 20ms between waypoints


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


enum FiniteState
{
	fs_start,
	fs_keep_lane
};

