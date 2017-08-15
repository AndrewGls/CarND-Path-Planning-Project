#include "LaneKeeping.h"
#include "Types.h"
#include "TrajectoryPool.h"
#include "LaneChanging.h"
#include <iostream>


using namespace std;

using Eigen::VectorXd;
using Eigen::MatrixXd;


std::tuple<VehicleState*, TrajectoryPtr> LineKeeping::OptimalTrajectory( const Eigen::VectorXd& currStateX6,
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
	TOtherCarsTrajectory CarsOnThisLaneTrajectories = rSF.GetLeadingCarsTrajectoryInLane(currStateX6, nCurrentLane, m_HorizontPrediction, m_TimeStep);

	// Generates trajectory to stay in the lane.
	if (bOnKeepLane)
	{
		pool.SetOtherCars(CarsOnThisLaneTrajectories);
		for (double v = 0; v < m_SpeedLimit; v += 1)
		{
			for (double T = 1; T < maxT; T += 1)
			{
				pool.AddTrajectory(Trajectory::VelocityKeeping_STrajectory(currStateX6, currentD, v, currTime, T, 0));
			}
		}
	}

	// Changing lane to right trajectory
	if (nCurrentLane <= 1 && bOnLaneChangingRight)
	{
		const int nTargetLane = nCurrentLane + 1;
		const double targetD = Utils::LaneNumberToD(nTargetLane);

		if (!bChangeLineTestMode)
		{
			pool.SetOtherCars(CarsOnThisLaneTrajectories);
			pool.AddOtherCars(rSF.GetOtherCarsTrajectoryInLane(currStateX6, nTargetLane, m_HorizontPrediction, m_TimeStep));
		}
		else
		{
//			std::cout << " Traj Move to right: " << nTargetLane << std::endl;
			pool.SetOtherCars(std::vector<Eigen::MatrixXd>());
		}

		for (double v = 0; v < m_SpeedLimit; v += 1)
		{
			for (double T = 1; T < maxT; T += 1)
			{
				TrajectoryPtr pTraj = Trajectory::VelocityKeeping_STrajectory(currStateX6, targetD, v, currTime, T, LaneChangingTime);
				pool.AddTrajectory(pTraj);
			}
		}
	}

	// Changing lane to left trajectory
	if (nCurrentLane >= 1 && bOnLaneChangingLeft)
	{
		const int nTargetLane = nCurrentLane - 1;
		const double targetD = Utils::LaneNumberToD(nTargetLane);

		if (!bChangeLineTestMode)
		{
			pool.SetOtherCars(CarsOnThisLaneTrajectories);
			pool.AddOtherCars(rSF.GetOtherCarsTrajectoryInLane(currStateX6, nTargetLane, m_HorizontPrediction, m_TimeStep));
		}
		else
		{
//			std::cout << " Traj Move to left: " << nTargetLane << std::endl;
			pool.SetOtherCars(std::vector<Eigen::MatrixXd>());
		}

		for (double v = 0; v < m_SpeedLimit; v += 1)
		{
			for (double T = 1; T < maxT; T += 1)
			{
				TrajectoryPtr pTraj = Trajectory::VelocityKeeping_STrajectory(currStateX6, targetD, v, currTime, T, LaneChangingTime);
				pool.AddTrajectory(pTraj);
			}
		}
	}


	VehicleState* pNextState = this;
	TrajectoryPtr pOptimalTraj = pool.OptimalTrajectory();
	const auto targetD = pOptimalTraj->GetTargetD();

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
		<< " v: " << pOptimalTraj->TragetVelocity()
		<< " T: " << pOptimalTraj->GetDurationS()
		<< " startD: " << pOptimalTraj->GetStartD()
		<< " endD: " << pOptimalTraj->GetTargetD()
		<< " C: " << pOptimalTraj->GetTotalCost()
		<< " DC: " << pOptimalTraj->GetSafetyDistanceCost()
		<< " VC: " << pOptimalTraj->GetVelocityCost()
		<< " JC: " << pOptimalTraj->GetJerkCost()
		<< endl;
	cout << "------- Best ----------" << endl;
#endif // #ifdef VERBOSE_BEST_TRAJECTORY


//	pOptimalTraj->PrintInfo();

	return tuple<VehicleState*, TrajectoryPtr>(pNextState, pOptimalTraj);
}


