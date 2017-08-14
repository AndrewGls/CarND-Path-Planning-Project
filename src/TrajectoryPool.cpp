#include "TrajectoryPool.h"
#include <iostream>
#include <algorithm>


using namespace std;


TrajectoryPool::TrajectoryPool(double speedLimit, double timeHorizon)
	: m_SpeedLimit(speedLimit)
	, m_HorizontPrediction(timeHorizon)
{
}


void TrajectoryPool::SetOtherCars(const TOtherCarsTrajectory& otherTrajectories)
{
	m_otherTrajectories = otherTrajectories;
}


void TrajectoryPool::AddOtherCars(const TOtherCarsTrajectory& otherTrajectories)
{
	m_otherTrajectories.insert(m_otherTrajectories.end(), otherTrajectories.begin(), otherTrajectories.end());
}


void TrajectoryPool::AddTrajectory(TrajectoryPtr traj)
{
	m_pool.push_back(traj);
	CalcTrajectoryCost(traj);
}


TrajectoryPtr TrajectoryPool::OptimalTrajectory() const
{
	TrajectoryPtr pTraj;
	double minCost = 1e10;// std::numeric_limits<double>::max();

	for (auto it = m_pool.cbegin(); it != m_pool.cend(); ++it)
	{
		const auto cost = (*it)->GetTotalCost();
		if (cost < minCost)
		{
			minCost = cost;
			pTraj = *it;
		}
	}

	return pTraj;
}

void TrajectoryPool::CalcTrajectoryCost(TrajectoryPtr pTraj)
{
	double maxJs, maxJd;
	double JerkCost = JerkCostWeight * pTraj->CalcJerkCost(m_HorizontPrediction, maxJs, maxJd);
	const double AccelCost = AccelCostWeight * pTraj->CalcAccelCost(m_HorizontPrediction);
	const double LaneOffsetCost = LaneOffsetCostWeight * pTraj->CalcLaneOffsetCost(m_HorizontPrediction);

	const std::pair<double, double>& MinMaxVelocity = pTraj->MinMaxVelocity_S();
	double VelocityCost = VelocityCostWeight * pTraj->CalcVelocityCost(m_SpeedLimit, m_HorizontPrediction);

	// Penalize invalid trajectories!
	if (MinMaxVelocity.first < 0 || MinMaxVelocity.second > m_SpeedLimit)
	{
		VelocityCost += 1000;
	}

	if (std::abs(maxJs) > m_MaxJerkS || std::abs(maxJd) > m_MaxJerkS)
	{
		JerkCost += 1000;
	}

	double MaxSaferyDistCost = 0;

	for (const auto& otherTraj : m_otherTrajectories)
	{
		const double safetyDistCost = pTraj->CalcSaferyDistanceCost(otherTraj, m_HorizontPrediction);
		MaxSaferyDistCost = std::max(MaxSaferyDistCost, safetyDistCost);
	}

	double SaferyDistCost = SaferyDistCostWeight * MaxSaferyDistCost;

	pTraj->SetJerkCost(JerkCost);
	pTraj->SetAccelCost(AccelCost);
	pTraj->SetVelocityCost(VelocityCost);
	pTraj->SetSafetyDistanceCost(SaferyDistCost);
	pTraj->SetLaneOffsetCost(LaneOffsetCost);

#ifdef VERBOSE_TRAJECTORIES
	Eigen::VectorXd currStateV6 = pTraj->GetStartState();
	cout //<< "sD: " << pTraj->GetStartD()
		//<< " eD: " << pTraj->GetTargetD()
		<< " v: " << pTraj->TragetVelocity()
		<< " T: " << pTraj->GetDurationS()
		<< " C: " << pTraj->GetTotalCost()
		<< " Jc: " << pTraj->GetJerkCost()
		<< " Ac: " << pTraj->GetAccelCost()
		<< " Vc: " << pTraj->GetVelocityCost()
		<< " DC: " << SaferyDistCost
		<< " SH: " << pTraj->getLaneOffsetCost()
		<< " Vmin: " << MinMaxVelocity.first
		<< " Vmax: " << MinMaxVelocity.second
//			<< " cars: " << m_otherTrajectories.size()
//		<< " D_MIN: " << pTraj->DE_D_MIN
//		<< " D_MAX: " << pTraj->DE_D_MAX
<< endl;
#endif // VERBOSE_TRAJECTORIES

//	pTraj->PrintInfo();

}

