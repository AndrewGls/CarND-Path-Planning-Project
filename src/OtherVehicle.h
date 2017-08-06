#pragma once

#include "Utils.hpp"
#include "Trajectory.h"

class OtherVehicle
{
public:
	OtherVehicle(int id = -1, double x = 0, double y = 0, double v = 0, double s = 0, double d = 0);

	int    get_id() const { return m_id; }
	double get_s() const { return m_s; }
	double get_d() const { return m_d; }
	double get_v() const { return m_v; }

	double get_x() const { return m_x; }
	double get_y() const { return m_y; }

	int    get_lane() const { return Utils::GetLaneNumberFor(m_d); }
	bool   isInlane (int lane) const { return lane == get_lane() && lane != -1;  }

	TrajectoryPtr PredictedTrajectory (double currTime, double timeDuration) const;

private:
	int m_id;
	double m_x;
	double m_y;
	double m_v;
	double m_s;
	double m_d;
};


//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------

inline OtherVehicle::OtherVehicle (int id , double x, double y, double v, double s, double d)
	: m_id(id)
	, m_x(x)
	, m_y(y)
	, m_v(v)
	, m_s(s)
	, m_d(d)
{
}

//---------------------------------------------------------------------------------------
inline TrajectoryPtr OtherVehicle::PredictedTrajectory (double currTime, double timeDuration) const
{
	return Trajectory::ConstartVelocity_STrajectory (m_s, m_d, m_v, currTime, timeDuration);
}
