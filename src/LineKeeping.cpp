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
	const double D0 = saferyDistance(currVelosity);

	constexpr double velocity_step = 1; // m/s
	constexpr double T_step = 1;		// s
	constexpr double maxT = 10;			// s

	const double timeStep = m_delta_t;  // time step, used to calculate trajectory cost function.

	TrajectoryPtr pOptimalTraj;
	TrajectoryPool pool;

	for (double v = 0; v < m_SpeedLimit; v += velocity_step)
	{
		for (double T = 1; T < maxT; T += T_step)
		{
			TrajectoryPtr pTraj = Trajectory::VelocityKeeping_STrajectory(currStateX6, v, currTime, T);
			const double TimeCost = TimeCostW * T;
			const double JerkCost = JerkCostW * pTraj->jerkCost_SD (timeStep);
			const std::pair<double, double>& MinMaxVelocity = pTraj->MinMaxVelocity_S (timeStep);
			double VelocityCost = std::pow (v - m_SpeedLimit, 2);

			// Penalize invalid trajectories!
			if (MinMaxVelocity.first < 0 || MinMaxVelocity.second > m_SpeedLimit)
			{
				VelocityCost += 1000;
			}

			pTraj->addCost(JerkCost + TimeCost + VelocityCost);

			cout << "s: " << currStateX6(0)
				//<< " d: " << currStateX6(3)
				<< " v: " << v
				<< " T: " << T
				<< " Jc: " << JerkCost
				<< " Vc: " << VelocityCost
				<< " Tc: " << TimeCost
				<< " C: " << pTraj->getCost()
				<< " Vmin: " << MinMaxVelocity.first
				<< " Vmax: " << MinMaxVelocity.second
				<< endl;

			pool.add(pTraj);
		}
	}

	pOptimalTraj = pool.optimalTrajectory();

	cout << "------- Best ----------" << endl;
	cout << "s: " << currStateX6(0)
		//<< " d: " << currStateX6(3)
		<< " v: " << pOptimalTraj->DE_V
		<< " T: " << pOptimalTraj->getDuration()
		<< endl;

	return pOptimalTraj;
}


