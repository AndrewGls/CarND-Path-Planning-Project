#pragma once

#include "Utils.hpp"
#include "Trajectory.h"


class OtherVehicle
{
public:
	OtherVehicle(double s = 0, double d = 0, double vs = 0);

	int    get_lane() const { return Utils::DtoLaneNumber(get_d()); }
	bool   isInlane(int lane) const { return lane == get_lane() && lane != -1; }

	double get_s() const { return m_x(0); }
	double get_d() const { return m_x(1); }
	double get_v() const { return m_x(2); }

	void predict(double deltaTime)					{ m_x(0) += m_x(2) * deltaTime; }
	void update(double s, double d, double vs)		{ m_x << s, d, vs, 0; }

	// Generates trajectory as matrix of rows [s, d, vs, vd].
	// Number of rows = duration / delataTime.
	Eigen::MatrixXd PredictedTrajectory(double delataTime, double duration);

private:
	Eigen::VectorXd m_x;
};


using TOtherVehicles = std::vector<OtherVehicle>;
using TOtherCarsTrajectory = std::vector<Eigen::MatrixXd>;


//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------

inline OtherVehicle::OtherVehicle (double s, double d, double vs)
{
	m_x = Eigen::VectorXd::Zero(4);
	m_x << s, d, vs, 0;
}

//---------------------------------------------------------------------------------------

inline Eigen::MatrixXd OtherVehicle::PredictedTrajectory(double delataTime, double duration)
{
	const auto k = static_cast<size_t>(duration / delataTime);
	Eigen::MatrixXd Result(k, 4);

	double s = m_x(0);

	for (size_t i = 0; i < k; i++)
	{
		s += m_x(2) * delataTime;
		Result.row(i) << s, m_x(1), m_x(2), 0;
	}

	return Result;
}
