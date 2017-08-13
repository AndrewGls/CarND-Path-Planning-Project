#pragma once

#include "Utils.hpp"
#include "Trajectory.h"
#include "KalmanFilter.h"


//#define ENABLE_KALMAN_FILTER

class OtherVehicle
{
public:
	OtherVehicle(double s = 0, double d = 0, double vs = 0);

	int    get_lane() const { return Utils::DtoLaneNumber(get_d()); }
	bool   isInlane(int lane) const { return lane == get_lane() && lane != -1; }

	double get_s() const;
	double get_d() const;
	double get_v() const;

	void predict(double deltaTime);
	void update(double s, double d, double vs);

	// Generates trajectory as matrix of rows [s, d, vs, vd].
	// Number of rows = duration / delataTime.
	Eigen::MatrixXd PredictedTrajectory(double delataTime, double duration);

private:
#ifdef ENABLE_KALMAN_FILTER
	KalmanFilter m_kalmanFilter;
#endif
	Eigen::VectorXd m_x;
};


using TOtherVehicles = std::vector<OtherVehicle>;
using TOtherCarsTrajectory = std::vector<Eigen::MatrixXd>;


//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------

inline OtherVehicle::OtherVehicle (double s, double d, double vs)
#ifdef ENABLE_KALMAN_FILTER
	: m_kalmanFilter(s, d, vs)
#endif
{
	m_x = Eigen::VectorXd::Zero(4);
	m_x << s, d, vs, 0;
}

//---------------------------------------------------------------------------------------

inline double OtherVehicle::get_s() const
{
#ifdef ENABLE_KALMAN_FILTER
	return m_kalmanFilter.s();
#else
	return m_x(0);
#endif
}

inline double OtherVehicle::get_d() const
{
#ifdef ENABLE_KALMAN_FILTER
	return m_kalmanFilter.d();
#else
	return m_x(1);
#endif
}

inline double OtherVehicle::get_v() const
{
#ifdef ENABLE_KALMAN_FILTER
	return m_kalmanFilter.vs();
#else
	return m_x(2);
#endif
}

//---------------------------------------------------------------------------------------

#ifdef ENABLE_KALMAN_FILTER
inline void OtherVehicle::predict(double deltaTime)
{
	m_kalmanFilter.PredictStep(deltaTime);
}
#else
inline void OtherVehicle::predict(double deltaTime)
{
	m_x(0) += m_x(2) * deltaTime;
}
#endif

inline void OtherVehicle::update(double s, double d, double vs)
{
#ifdef ENABLE_KALMAN_FILTER
	m_kalmanFilter.UpdateStep(s, d, vs);
#else
	m_x << s, d, vs, 0;
#endif
}

//---------------------------------------------------------------------------------------

#ifdef ENABLE_KALMAN_FILTER

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

#else

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

#endif // ENABLE_KALMAN_FILTER
