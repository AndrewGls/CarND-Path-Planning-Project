#include "SensorFusion.h"
#include "Types.h"
#include "Utils.hpp"
#include <algorithm>
#include <iostream>


using namespace std;


SensorFusion::SensorFusion(const vector<SensorFusionData>& sensorFusion, double delayTime, const HighwayMap& map)
{
	const auto count = sensorFusion.size();

	m_vehicles.resize(count);

#ifdef VERBOSE_OTHER_CARS_CHECK_TRANSFORM
	std::cout << "Other cars at: ";
#endif

	for (size_t i = 0; i < count; i++)
	{
		const SensorFusionData& sf = sensorFusion[i];

		// We expect that vehicle drives with constant velocity.
		const auto x = sf.x + sf.vx * delayTime;
		const auto y = sf.y + sf.vy * delayTime;
		const auto v = sqrt(sf.vx * sf.vx + sf.vy * sf.vy);
		const auto s = sf.s + v * delayTime;

		const Eigen::Vector2d sd = map.CalcFrenet(Point(x, y), s);

		double ddd = sd(1);
		if (ddd < 0)
		{
			int i = 0;
		}

#ifdef VERBOSE_OTHER_CARS_CHECK_TRANSFORM
		if (sd(1) < 0)
		{
		std::cout << " "
			<< x << ","
			<< y << ","
			<< sd(0) << ","
			<< sd(1)
			<< std::endl;
		}
#endif

		m_vehicles[i] = OtherVehicle(sf.id, x, y, v, sd(0), sd(1));
	}
}

TOtherVehicles SensorFusion::getLeadingVehiclesInLane (const Eigen::VectorXd& sdcStateV6) const
{
	TOtherVehicles leading_cars;
	const auto count = m_vehicles.size();

	leading_cars.reserve(count);

	const auto s = sdcStateV6(0);
	const auto d = sdcStateV6(3);
	const int lane = Utils::GetLaneNumberFor(d);

	for (size_t i = 0; i < count; i++)
	{
		const auto& car = m_vehicles[i];
		if (car.get_s() > s && car.isInlane(lane))
		{
			leading_cars.push_back(car);
		}
#ifdef VERBOSE_OTHER_IGNORED_CARS
		else
		{
			cout << "Ignore car id: " << car.get_id()
				<< " s: " << car.get_s()
				<< " d: " << car.get_d()
				<< endl;
		}
#endif // VERBOSE_OTHER_IGNORED_CARS
	}

	// sorts by ascending S
	sort(leading_cars.begin(), leading_cars.end(), [](const OtherVehicle& l, const OtherVehicle& r) { return l.get_s() < r.get_s(); });

#ifdef VERBOSE_OTHER_LEADING_CARS
	for (const auto car : leading_cars)
	{
		cout << "---------------------" << endl;
		cout << "Detected car id: " << car.get_id()
			<< " s: " << car.get_s()
			<< " d: " << car.get_d()
			<< endl;
	}
#endif // VERBOSE_OTHER_LEADING_CARS

	return leading_cars;
}

