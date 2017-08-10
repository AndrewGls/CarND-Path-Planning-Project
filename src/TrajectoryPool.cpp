#include "TrajectoryPool.h"
#include <iostream>
#include <algorithm>


using namespace std;


TrajectoryPool::TrajectoryPool(const TOtherCarsTrajectory& otherTrajectories, double speedLimit, double timeHorizon)
	: m_otherTrajectories(otherTrajectories)
	, m_SpeedLimit(speedLimit)
	, m_HorizontPrediction(timeHorizon)
{
}


void TrajectoryPool::addTrajectory(TrajectoryPtr traj)
{
	m_pool.push_back(traj);
	calcTrajectoryCost(traj);
}


TrajectoryPtr TrajectoryPool::optimalTrajectory() const
{
	TrajectoryPtr pTraj;
	double minCost = 1e10;// std::numeric_limits<double>::max();

	for (auto it = m_pool.cbegin(); it != m_pool.cend(); ++it)
	{
		const auto cost = (*it)->getTotalCost();
		if (cost < minCost)
		{
			minCost = cost;
			pTraj = *it;
		}
	}

	return pTraj;
}

void TrajectoryPool::calcTrajectoryCost(TrajectoryPtr pTraj)
{
	double maxJs, maxJd;
	const double TimeCost = TimeCostWeight * pTraj->getDuration();
	double       JerkCost = JerkCostWeight * pTraj->jerkCost_SD(m_HorizontPrediction, maxJs, maxJd);

	const std::pair<double, double>& MinMaxVelocity = pTraj->MinMaxVelocity_S();
	double VelocityCost = VelocityCostWeight * pTraj->velocityCost_S(m_SpeedLimit, m_HorizontPrediction);

	// Penalize invalid trajectories!
/*	if (MinMaxVelocity.first < 0 || MinMaxVelocity.second > m_SpeedLimit)
	{
		VelocityCost += 1000;
	}
	if (maxJs > m_MaxJerkS)
	{
		JerkCost += 1000;
	}*/

	double MaxSaferyDistCost = 0;

	for (const auto& otherTraj : m_otherTrajectories)
	{
		const double safetyDistCost = pTraj->calcSaferyDistanceCost(otherTraj, m_HorizontPrediction);
		MaxSaferyDistCost = std::max(MaxSaferyDistCost, safetyDistCost);
	}

	pTraj->setTimeCost(TimeCost);
	pTraj->setJerkCost(JerkCost);
	pTraj->setVelocityCost(VelocityCost);
	pTraj->setSafetyDistanceCost(SaferyDistCostWeight * MaxSaferyDistCost);


#ifdef VERBOSE_LINE_KEEPING
	Eigen::VectorXd currStateV6 = pTraj->getStartState();
	cout << "s: " << pTraj->getStartState()(0)
		//<< " d: " << currStateX6(3)
		<< " v: " << pTraj->DE_V
		<< " T: " << pTraj->getDuration()
		<< " Jc: " << JerkCost
		<< " Vc: " << VelocityCost
		<< " cars: " << m_otherTrajectories.size()
		//				<< " Tc: " << TimeCost
		<< " C: " << pTraj->getTotalCost()
		<< " DC: " << MaxSaferyDistCost
		//				<< " Vmin: " << MinMaxVelocity.first
		//				<< " Vmax: " << MinMaxVelocity.second
		<< endl;
#endif // VERBOSE_LINE_KEEPING
}

