#include "LineKeeping.h"
#include "Types.h"
#include "TrajectoryPool.h"
#include <iostream>


using namespace std;

using Eigen::VectorXd;
using Eigen::MatrixXd;


std::tuple<VehicleState*, TrajectoryPtr> LineKeeping::optimalTrajectory( const Eigen::VectorXd& currStateX6,
																		double currTime,
																		const SensorFusion& sensFusion )
{
	// https://d17h27t6h515a5.cloudfront.net/topher/2017/July/595fd482_werling-optimal-trajectory-generation-for-dynamic-street-scenarios-in-a-frenet-frame/werling-optimal-trajectory-generation-for-dynamic-street-scenarios-in-a-frenet-frame.pdf

	// Start state: [s0, s0_d, s0_dd] at t0
	// End state:   [s1_d, s1_dd] at t1 = t0 + T
	// st_d = target velocity.
	// Task: generate optimal longitudinal trajectory set of quartic polynomials by
	// varying the end constraints by ∆s_d[i] and T[j] according to: [ s1_d, s1+dd, T][ij] = [[st_d + ∆s_d[i]], 0, T[j] ]

	const double Tmax = 10;

	const double nCurrS = currStateX6(0);
	const double nCurrD = currStateX6(3);
	const int nCurrLane = Utils::getLaneNumberForD(nCurrD);

	// Generates trajectory to stay in the lane.
	TOtherCarsTrajectory otherTrajectories = sensFusion.GetOtherCarTrajectoryInLane(currStateX6, nCurrLane, m_HorizontPrediction, m_TimeStep);
	TrajectoryPool pool(otherTrajectories, m_SpeedLimit, m_HorizontPrediction);
	for (double v = 0; v < m_SpeedLimit; v += 1)
	{
		for (double T = 1; T < Tmax; T += 1)
		{
			TrajectoryPtr pTraj = Trajectory::VelocityKeeping_STrajectory(currStateX6, nCurrD, v, currTime, T, 0);
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
		<< " T: " << pOptimalTraj->getDurationS()
		<< endl;
	cout << "------- Best ----------" << endl;
#endif // #ifdef VERBOSE_BEST_TRAJECTORY

//	pOptimalTraj->PrintInfo();

	return tuple<VehicleState*, TrajectoryPtr>(this, pOptimalTraj);
}


