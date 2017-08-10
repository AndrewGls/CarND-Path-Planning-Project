#include "LineKeeping.h"
#include "Types.h"
#include "TrajectoryPool.h"
#include <iostream>


using namespace std;

using Eigen::VectorXd;
using Eigen::MatrixXd;


LineKeeping::LineKeeping()
{

}

LineKeeping::~LineKeeping()
{
}

TrajectoryPtr LineKeeping::optimalTrajectory( const Eigen::VectorXd& currStateX6,
											  double currTime,
											  const SensorFusion& sensFusion )
{
	// https://d17h27t6h515a5.cloudfront.net/topher/2017/July/595fd482_werling-optimal-trajectory-generation-for-dynamic-street-scenarios-in-a-frenet-frame/werling-optimal-trajectory-generation-for-dynamic-street-scenarios-in-a-frenet-frame.pdf

	// Start state: [s0, s0_d, s0_dd] at t0
	// End state:   [s1_d, s1_dd] at t1 = t0 + T
	// st_d = target velocity.
	// Task: generate optimal longitudinal trajectory set of quartic polynomials by
	// varying the end constraints by ∆s_d[i] and T[j] according to: [ s1_d, s1+dd, T][ij] = [[st_d + ∆s_d[i]], 0, T[j] ]

	const double currS = currStateX6(0);
	const double currVelosity = currStateX6(1);

	const double currD = currStateX6(3);
	const int currLane = Utils::getLaneNumberForD(currD);

	constexpr double velocity_step = 1; // m/s
	constexpr double T_step = 1;		// s
	constexpr double maxT = 10;			// s

	const double timeStep = m_delta_t;  // time step, used to calculate trajectory cost function.

	auto nearestVehicles = sensFusion.getNearestVehiclesInLane(currStateX6);
	TOtherCarsTrajectory otherTrajectories;

	for (OtherVehicle& otherCar : nearestVehicles)
	{
		otherTrajectories.push_back(otherCar.PredictedTrajectory(0.1, m_HorizontPrediction));
	}


	TrajectoryPool pool(otherTrajectories, m_SpeedLimit, m_HorizontPrediction);

	for (double v = 0; v < m_SpeedLimit; v += velocity_step)
	{
		for (double T = 1; T < maxT; T += T_step)
		{
			if (v == 1 && T == 1)
			{
				int i = 0;
			}
			TrajectoryPtr pTraj = Trajectory::VelocityKeeping_STrajectory(currStateX6, currD, v, currTime, T);
			pool.addTrajectory(pTraj);
		}
	}

	TrajectoryPtr pOptimalTraj = pool.optimalTrajectory();

#ifdef VERBOSE_BEST_TRAJECTORY
	cout << "------- Best ----------" << endl;
	cout << "s: " << currStateX6(0)
		//<< " d: " << currStateX6(3)
		<< " MinCost: " << pOptimalTraj->getTotalCost()
		<< " v: " << pOptimalTraj->DE_V
		<< " T: " << pOptimalTraj->getDuration()
		<< endl;
#endif // #ifdef VERBOSE_BEST_TRAJECTORY

	return pOptimalTraj;
}


