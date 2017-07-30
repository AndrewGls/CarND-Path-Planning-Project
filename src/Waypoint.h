#pragma once


struct Point
{
	double x;
	double y;

	Point(double x = 0, double y = 0) : x(x), y(y) {}
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
