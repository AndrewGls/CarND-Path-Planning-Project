#include "SensorFusion.h"
#include "Types.h"
#include "Utils.hpp"
#include <algorithm>
#include <iostream>
#include <vector>


using namespace std;

void SensorFusion::predict(double deltaTime)
{
	for (auto& elem : m_mapVehicles)
		elem.second.predict(deltaTime);
}


void SensorFusion::update(const vector<SensorFusionData>& sensorFusion, double currS, const Waypoints& map)
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
			it->second.update(s, sd(1), v);

		m_mapTrackedCars[sf.id] = true;
	}

	for (const auto& elem : m_mapTrackedCars)
	{
		if (!elem.second)
			m_mapVehicles.erase(elem.first);
	}
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


TOtherVehicles SensorFusion::GetLeadingCarsInLane (const Eigen::VectorXd& sdcStateV6, bool bOnlyNearest) const
{
	TOtherVehicles leading_cars;
	leading_cars.reserve(20);

	const auto s = sdcStateV6(0);
	const auto d = sdcStateV6(3);
	const int lane = Utils::DtoLaneNumber(d);

	for (const auto& elem : m_mapVehicles)
	{
		const auto& car = elem.second;
		if (car.get_s() > s && car.isInlane(lane))
		{
			leading_cars.push_back(car);
		}
#ifdef VERBOSE_OTHER_IGNORED_CARS
		else
		{
			cout << "Ignored car: ("<< car.get_s() << "," << car.get_d() << ")" << endl;
		}
#endif // VERBOSE_OTHER_IGNORED_CARS
	}

	// sorts by ascending S
	sort(leading_cars.begin(), leading_cars.end(), [](const OtherVehicle& l, const OtherVehicle& r) { return l.get_s() < r.get_s(); });
	if (bOnlyNearest)
		leading_cars.resize(1);

#ifdef VERBOSE_OTHER_LEADING_CARS
	for (const auto car : leading_cars)
	{
		cout << "---------------------" << endl;
		cout << "Detected car in front at: (" << car.get_s() << "," << car.get_d() << ")"
			<< " DIST: " << leading_cars[0].get_s() - sdcStateV6(0)
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
		if (car.isInlane(nLane) && Utils::distance(car.get_s(), s) < deltaS)
		{
			leading_cars.push_back(car);
		}
	}

#ifdef VERBOSE_OTHER_LEADING_CARS
	cout << "---------------------" << endl;
	for (const auto car : leading_cars)
	{
		cout << "Detected car in front at: (" << car.get_s() << "," << car.get_d() << ")" << "),      S:" << s << std::endl;
	}
#endif // VERBOSE_OTHER_LEADING_CARS

	return leading_cars;
}
