#include "SensorFusion.h"
#include "Types.h"
#include "Utils.hpp"
#include <algorithm>
#include <iostream>


using namespace std;


void SensorFusion::predict(double deltaTime)
{
	for (auto& elem : m_mapVehicles)
		elem.second.predict(deltaTime);
}


void SensorFusion::update(const vector<SensorFusionData>& sensorFusion, double currS, const HighwayMap& map)
{
	m_mapUpdate.clear();
	for (const auto& elem : m_mapVehicles)
		m_mapUpdate[elem.first] = false;

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

		m_mapUpdate[sf.id] = true;
	}

	for (const auto& elem : m_mapUpdate)
	{
		if (!elem.second)
			m_mapVehicles.erase(elem.first);
	}
}


TOtherVehicles SensorFusion::getLeadingVehiclesInLane (const Eigen::VectorXd& sdcStateV6) const
{
	TOtherVehicles leading_cars;
	leading_cars.reserve(20);

	const auto s = sdcStateV6(0);
	const auto d = sdcStateV6(3);
	const int lane = Utils::getLaneNumberForD(d);

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

#ifdef VERBOSE_OTHER_LEADING_CARS
	for (const auto car : leading_cars)
	{
		cout << "---------------------" << endl;
		cout << "Detected car in front at: (" << car.get_s() << "," << car.get_d() << ")" << endl;
	}
#endif // VERBOSE_OTHER_LEADING_CARS

	return leading_cars;
}

TOtherVehicles SensorFusion::getNearestVehiclesInLane(const Eigen::VectorXd& sdcStateV6, int lane, double deltaS) const
{
	TOtherVehicles leading_cars;
	leading_cars.reserve(20);

	const auto s = sdcStateV6(0);

	for (const auto& elem : m_mapVehicles)
	{
		const auto& car = elem.second;
		if (car.isInlane(lane) && car.get_s() > s - deltaS && car.get_s() < s + deltaS)
		{
			leading_cars.push_back(car);
		}
	}

	return leading_cars;
}
