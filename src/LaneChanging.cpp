#include "LaneChanging.h"
#include "TrajectoryPool.h"
#include <vector>

using namespace std;


LaneChanging::LaneChanging(int nStartLane, int nTargetLane)
	: m_nStartLane(nStartLane)
	, m_nTargetLane(nTargetLane)
{

}

tuple<VehicleState*, TrajectoryPtr> LaneChanging::optimalTrajectory(const Eigen::VectorXd& currStateV6,
											double currTime,
											const SensorFusion& rSF)
{
#ifdef VERBOSE_STATE
	std::cout << " Change Lane State " << std::endl;
#endif

	const int maxT = 10;

	const double currentD = currStateV6(3);
	const double startD = Utils::LaneNumberToD(m_nStartLane);
	const double targetD = Utils::LaneNumberToD(m_nTargetLane);
	const double changingLaneTime = 4;

	TrajectoryPool pool(m_SpeedLimit, m_HorizontPrediction);
	
	pool.setOtherCars(rSF.GetOtherCarsTrajectoryInLane(currStateV6, m_nTargetLane, m_HorizontPrediction, m_TimeStep));
	pool.addOtherCars(rSF.GetOtherCarsTrajectoryInLane(currStateV6, m_nStartLane, m_HorizontPrediction, m_TimeStep));

	{
		for (double v = 0; v < m_SpeedLimit; v += 1)
		{
			for (double T = 1; T < maxT; T += 1)
			{
				TrajectoryPtr pTraj = Trajectory::VelocityKeeping_STrajectory(currStateV6, targetD, v, currTime, T, changingLaneTime);
				pool.addTrajectory(pTraj);
			}
		}
	}


	VehicleState* pNextState = this;
	TrajectoryPtr pOptimalTraj = pool.optimalTrajectory();

	assert(targetD == pOptimalTraj->getTargetD());

	if (std::abs(targetD - currentD) < 0.1)
	{
#ifdef VERBOSE_STATE
		std::cout << " Remove Change Lane State " << std::endl;
#endif
		pNextState = nullptr;
	}

#ifdef VERBOSE_BEST_TRAJECTORY
	cout << "------- Best ----------" << endl;
	cout << " CHANGE: " << std::abs(targetD - currentD)
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

//	pOptimalTraj->PrintInfo();

	return tuple<VehicleState*, TrajectoryPtr>(pNextState, pOptimalTraj);
}
