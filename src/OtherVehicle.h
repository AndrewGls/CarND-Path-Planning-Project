#pragma once

#include "Utils.hpp"
#include "Trajectory.h"
#include "KalmanFilter.h"


class OtherVehicle
{
public:
	OtherVehicle(double s = 0, double d = 0, double vs = 0);

	int    get_lane() const { return Utils::DtoLaneNumber(get_d()); }
	bool   isInlane(int lane) const { return lane == get_lane() && lane != -1; }

	double get_s() const { return m_kalmanFilter.s(); }
	double get_d() const { return m_kalmanFilter.d(); }
//	double get_v() const { return m_kalmanFilter.vs(); }

	void predict(double deltaTime) { m_kalmanFilter.PredictStep(deltaTime); }
	void update(double s, double d, double vs) { m_kalmanFilter.UpdateStep(s, d, vs); }

	// Generates trajectory as matrix of rows [s, d, vs, vd].
	// Number of rows = duration / delataTime.
	Eigen::MatrixXd PredictedTrajectory(double delataTime, double duration);

private:
	KalmanFilter m_kalmanFilter;
};


using TOtherVehicles = std::vector<OtherVehicle>;
using TOtherCarsTrajectory = std::vector<Eigen::MatrixXd>;


//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------

inline OtherVehicle::OtherVehicle (double s, double d, double vs)
	: m_kalmanFilter(s, d, vs)
{
}


//---------------------------------------------------------------------------------------

inline Eigen::MatrixXd OtherVehicle::PredictedTrajectory(double delataTime, double duration)
{
	m_kalmanFilter.SaveState();

	const auto k = static_cast<size_t>(duration / delataTime);
	Eigen::MatrixXd Result(k, 4);

	for (size_t i = 0; i < k; i++)
	{
		Result.row(i) << m_kalmanFilter.x().transpose();
		m_kalmanFilter.PredictStep(delataTime);
	}

	m_kalmanFilter.RestoreState();

	return Result;
}
