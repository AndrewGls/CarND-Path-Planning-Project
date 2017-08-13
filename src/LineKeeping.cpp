#include "LineKeeping.h"
#include "Types.h"
#include "TrajectoryPool.h"
#include "LaneChanging.h"
#include <iostream>


using namespace std;

using Eigen::VectorXd;
using Eigen::MatrixXd;


std::tuple<VehicleState*, TrajectoryPtr> LineKeeping::optimalTrajectory( const Eigen::VectorXd& currStateX6,
																		double currTime,
																		const SensorFusion& rSF )
{
	bool bOnKeepLane = true;
	bool bOnLaneChangingLeft = true;
	bool bOnLaneChangingRight = true;
	bool bChangeLineTestMode = false;

	// https://d17h27t6h515a5.cloudfront.net/topher/2017/July/595fd482_werling-optimal-trajectory-generation-for-dynamic-street-scenarios-in-a-frenet-frame/werling-optimal-trajectory-generation-for-dynamic-street-scenarios-in-a-frenet-frame.pdf

	// Start state: [s0, s0_d, s0_dd] at t0
	// End state:   [s1_d, s1_dd] at t1 = t0 + T
	// st_d = target velocity.
	// Task: generate optimal longitudinal trajectory set of quartic polynomials by
	// varying the end constraints by ∆s_d[i] and T[j] according to: [ s1_d, s1+dd, T][ij] = [[st_d + ∆s_d[i]], 0, T[j] ]

	const double maxT = 10;
	const double LaneChangingTime = 4;

	const double currentS = currStateX6(0);
	const double currentD = currStateX6(3);
	const int nCurrentLane = Utils::DtoLaneNumber(currentD);

	TrajectoryPool pool (m_SpeedLimit, m_HorizontPrediction);
	TOtherCarsTrajectory CarsOnThisLaneTrajectories = rSF.GetOtherCarsTrajectoryInLane(currStateX6, nCurrentLane, m_HorizontPrediction, m_TimeStep);

	// Generates trajectory to stay in the lane.
	if (bOnKeepLane)
	{
		pool.setOtherCars(CarsOnThisLaneTrajectories);
		for (double v = 0; v < m_SpeedLimit; v += 1)
		{
			for (double T = 1; T < maxT; T += 1)
			{
				TrajectoryPtr pTraj = Trajectory::VelocityKeeping_STrajectory(currStateX6, currentD, v, currTime, T, 0);
				pool.addTrajectory(pTraj);
			}
		}
	}

	// Changing lane to left trajectory
	if (nCurrentLane <= 1 && bOnLaneChangingLeft)
	{
		const int nTargetLane = nCurrentLane + 1;
		const double targetD = Utils::LaneNumberToD(nTargetLane);

		if (!bChangeLineTestMode)
		{
			pool.setOtherCars(CarsOnThisLaneTrajectories);
			pool.addOtherCars(rSF.GetOtherCarsTrajectoryInLane(currStateX6, nTargetLane, m_HorizontPrediction, m_TimeStep));
		}
		else
		{
//			std::cout << " Traj Move to left: " << nTargetLane << std::endl;
			pool.setOtherCars(std::vector<Eigen::MatrixXd>());
		}

		for (double v = 0; v < m_SpeedLimit; v += 1)
		{
			for (double T = 1; T < maxT; T += 1)
			{
				if (v == 15 && T == 5)
				{
					int i = 0;
				}

/*				std::cout << currStateX6 << std::endl;

				Eigen::VectorXd aCurrentStateTmp = Eigen::VectorXd::Zero(6);
				//		  aCurrentStateTmp << 124.932, 0, 0, -6.09322, 0, 0;
				aCurrentStateTmp << 124.932, 10, 0, -6.09322, 0, 0;

				TrajectoryPtr pTraj = Trajectory::VelocityKeeping_STrajectory(aCurrentStateTmp, targetD, 12, currTime, 8, LaneChangingTime);
				*/
				TrajectoryPtr pTraj = Trajectory::VelocityKeeping_STrajectory(currStateX6, targetD, v, currTime, T, LaneChangingTime);
				pool.addTrajectory(pTraj);
			}
		}
	}

	// Changing lane to right trajectory
	if (nCurrentLane >= 1 && bOnLaneChangingRight)
	{
		const int nTargetLane = nCurrentLane - 1;
		const double targetD = Utils::LaneNumberToD(nTargetLane);

		if (!bChangeLineTestMode)
		{
			pool.setOtherCars(CarsOnThisLaneTrajectories);
			pool.addOtherCars(rSF.GetOtherCarsTrajectoryInLane(currStateX6, nTargetLane, m_HorizontPrediction, m_TimeStep));
		}
		else
		{
//			std::cout << " Traj Move to right: " << nTargetLane << std::endl;
			pool.setOtherCars(std::vector<Eigen::MatrixXd>());
		}

		for (double v = 0; v < m_SpeedLimit; v += 1)
		{
			for (double T = 1; T < maxT; T += 1)
			{
				TrajectoryPtr pTraj = Trajectory::VelocityKeeping_STrajectory(currStateX6, targetD, v, currTime, T, LaneChangingTime);
				pool.addTrajectory(pTraj);
			}
		}
	}


	VehicleState* pNextState = this;
	TrajectoryPtr pOptimalTraj = pool.optimalTrajectory();
	const auto targetD = pOptimalTraj->getTargetD();

	if (currentD != targetD)
	{
#ifdef VERBOSE_STATE
		std::cout << " Add Change Lane State " << std::endl;
#endif
		pNextState = new LaneChanging (nCurrentLane, Utils::DtoLaneNumber(targetD));
	}


#ifdef VERBOSE_BEST_TRAJECTORY
	cout << "------- Best ----------" << endl;
	cout << " KEEP: " << std::abs(currentD - targetD)
		<< " v: " << pOptimalTraj->DE_V
		<< " T: " << pOptimalTraj->getDurationS()
		<< " startD: " << pOptimalTraj->getStartD()
		<< " endD: " << pOptimalTraj->getTargetD()
		<< " C: " << pOptimalTraj->getTotalCost()
		<< " DC: " << pOptimalTraj->getSafetyDistanceCost()
		<< " VC: " << pOptimalTraj->getVelocityCost()
		<< " JC: " << pOptimalTraj->getJerkCost()
		<< endl;
	cout << "------- Best ----------" << endl;
#endif // #ifdef VERBOSE_BEST_TRAJECTORY


	pOptimalTraj->PrintInfo();

	return tuple<VehicleState*, TrajectoryPtr>(pNextState, pOptimalTraj);
}


