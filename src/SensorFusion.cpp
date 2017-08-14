#include "SensorFusion.h"
#include "Types.h"
#include "Utils.hpp"
#include <algorithm>
#include <iostream>
#include <vector>


using namespace std;

void SensorFusion::Predict(double deltaTime)
{
	for (auto& elem : m_mapVehicles)
		elem.second.Predict(deltaTime);
}


void SensorFusion::Update(const vector<SensorFusionData>& sensorFusion, double currS, const Waypoints& map)
{
	m_mapTrackedCars.clear();
	for (const auto& elem : m_mapVehicles)
		m_mapTrackedCars[elem.first] = false;

	const auto count = sensorFusion.size();

#ifdef VERBOSE_OTHER_CARS_CHECK_TRANSFORM
	std::cout << "Other cars at: ";
#endif

	for (size_t i = 0; i < count; i++)
	{
		const SensorFusionData& sf = sensorFusion[i];

		const auto v = sqrt(sf.vx * sf.vx + sf.vy * sf.vy);
		const Eigen::Vector2d sd = map.CalcFrenet(Point(sf.x, sf.y), sf.s);
		double s = sd(0);

		const double max_s = map.max_s();
		const double max_s2 = max_s / 2;

		while (currS - s > max_s2) {
			s += max_s;
		}
		while (currS - s < -max_s2) {
			s -= max_s;
		}

		auto it = m_mapVehicles.find(sf.id);

		if (it == m_mapVehicles.end())
			m_mapVehicles [sf.id] = OtherVehicle(s, sd(1), v);
		else
			it->second.Update(s, sd(1), v);

		m_mapTrackedCars[sf.id] = true;
	}

	for (const auto& elem : m_mapTrackedCars)
	{
		if (!elem.second)
			m_mapVehicles.erase(elem.first);
	}
}

TOtherCarsTrajectory SensorFusion::GetLeadingCarsTrajectoryInLane(const Eigen::VectorXd& currStateV6, int nLane, double timeDuration, double timeStep) const
{
	TOtherCarsTrajectory trajectories;
	auto OtherCars = GetLeadingCarsInLane(currStateV6, nLane);
	for (OtherVehicle& car : OtherCars)
	{
		trajectories.push_back(car.PredictedTrajectory(timeStep, timeDuration));
	}
	return trajectories;
}

TOtherCarsTrajectory SensorFusion::GetOtherCarsTrajectoryInLane(const Eigen::VectorXd& currStateV6, int nLane, double timeDuration, double timeStep) const
{
	TOtherCarsTrajectory otherTrajectories;
	auto OtherCars = GetNearestCarsInLane(currStateV6, nLane);
	for (OtherVehicle& car : OtherCars)
	{
		otherTrajectories.push_back(car.PredictedTrajectory(timeStep, timeDuration));
	}
	return otherTrajectories;
}


TOtherVehicles SensorFusion::GetLeadingCarsInLane (const Eigen::VectorXd& sdcStateV6, int nLane, bool bOnlyNearest) const
{
	TOtherVehicles leading_cars;
	leading_cars.reserve(20);

	const auto s = sdcStateV6(0);

	for (const auto& elem : m_mapVehicles)
	{
		const auto& car = elem.second;
		if (car.Get_S() > s && car.IsInlane(nLane))
		{
			leading_cars.push_back(car);
		}
#ifdef VERBOSE_OTHER_IGNORED_CARS
		else
		{
			cout << "Ignored car: ("<< car.Get_S() << "," << car.Get_D() << ")" << endl;
		}
#endif // VERBOSE_OTHER_IGNORED_CARS
	}

	// sorts by ascending S
	sort(leading_cars.begin(), leading_cars.end(), [](const OtherVehicle& l, const OtherVehicle& r) { return l.Get_S() < r.Get_S(); });
	if (bOnlyNearest)
		leading_cars.resize(1);

#ifdef VERBOSE_OTHER_LEADING_CARS
	for (const auto car : leading_cars)
	{
		cout << "---------------------" << endl;
		cout << "Detected car in front at: (" << car.Get_S() << "," << car.Get_D() << ")"
			<< " DIST: " << leading_cars[0].Get_S() - sdcStateV6(0)
			<< endl;
		cout << "---------------------" << endl;
	}
#endif // VERBOSE_OTHER_LEADING_CARS

	return leading_cars;
}

TOtherVehicles SensorFusion::GetNearestCarsInLane(const Eigen::VectorXd& sdcStateV6, int nLane, double deltaS) const
{
	TOtherVehicles leading_cars;

	const double s = sdcStateV6(0);

	for (const auto& elem : m_mapVehicles)
	{
		const auto& car = elem.second;
		if (car.IsInlane(nLane) && Utils::distance(car.Get_S(), s) < deltaS)
		{
			leading_cars.push_back(car);
		}
	}

#ifdef VERBOSE_OTHER_LEADING_CARS
	cout << "---------------------" << endl;
	for (const auto car : leading_cars)
	{
		cout << "Detected car in front at: (" << car.Get_S() << "," << car.Get_D() << ")" << "),      S:" << s << std::endl;
	}
#endif // VERBOSE_OTHER_LEADING_CARS

	return leading_cars;
}
