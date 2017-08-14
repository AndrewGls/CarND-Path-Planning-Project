#include "Vehicle.h"
#include "Waypoints.h"
#include "SensorFusion.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Vector2d;


using namespace std;


//////////////////////////////////////////////////////////////////////////////////////////
Vehicle::Vehicle(const Waypoints& wps)
	: m_waypoints(wps)
	, m_isInitDone(false)
{
	m_currStateV6 = VectorXd::Zero(6);
}

//////////////////////////////////////////////////////////////////////////////////////////
void Vehicle::UpdateTrajectory(const CarLocalizationData& newState,
							   const vector<double>& previous_path_x,
							   const vector<double>& previous_path_y)
{
	const int nPredictionPathSize = int(m_predictionHorizont * 1000) / 20;
	const int nReactivePathSize = int(m_reactiveHorizont * 1000) / 20;

	m_next_x_vals.clear();
	m_next_y_vals.clear();

	if (!m_isInitDone)
	{
		// first time/reset
		const double velocity = newState.speed * 0.44704;  // convert MPH to m/sec !!!!
		m_currTime = 0;
		const Vector2d& initialFrenet = m_waypoints.CalcFrenet(Point(newState.x, newState.y), newState.s);
		m_currStateV6 << initialFrenet(0), velocity, 0., initialFrenet(1), 0., 0.;
		m_isInitDone = true;
	}

	const int nPrevPathSize = static_cast<int>(previous_path_x.size());
	int nPrevPredictionPathSize = nPrevPathSize - (nPredictionPathSize - nReactivePathSize);
	nPrevPredictionPathSize = max(0, nPrevPredictionPathSize);
	for (int i = 0; i < nPrevPredictionPathSize; ++i)
	{
		m_next_x_vals.push_back(previous_path_x[i]);
		m_next_y_vals.push_back(previous_path_y[i]);
	}

	// Predict position of vehicles after delay-time:
	const double delayTime = nPrevPredictionPathSize * Utils::delta_t;
	m_sensorFusion.Update(m_sensorFusionData, m_currStateV6(0), m_waypoints);
	m_sensorFusion.Predict(delayTime);

	TrajectoryPtr pTraj = m_behavior.OptimalTrajectory(m_currStateV6, m_currTime, m_sensorFusion);

#ifdef VERBOSE_NEXT_XY
	cout << "------- Driving path ----------" << endl;
#endif

	double currTime = m_currTime;
	Eigen::VectorXd currState = m_currStateV6;

	for (int i = nPrevPredictionPathSize; i < nPredictionPathSize; ++i)
	{
		if (i == nReactivePathSize)
		{
			m_currTime = currTime;
			m_currStateV6 = currState;
		}

		const auto pt = m_waypoints.GetXYInterpolated(currState(0), currState(3));
		currTime += Utils::delta_t;
		currState = pTraj->EvaluateStateAt(currTime);

		m_next_x_vals.push_back(pt.x);
		m_next_y_vals.push_back(pt.y);

#ifdef VERBOSE_NEXT_XY
		cout << "s: " << currState(0)
			<< " d: " << currState(3)
			<< " (" << pt.x << ", " << pt.y << ") "
			<< " v: " << currState(1)
			<< " a: " << currState(2);
		cout << endl;
#endif // VERBOSE_NEXT_XY
	}

	//assert(m_next_x_vals.size() == nPredictionPathSize);
}
