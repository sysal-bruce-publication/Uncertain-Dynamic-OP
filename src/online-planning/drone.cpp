#include "drone.h"

namespace DS = DroneSpec;

void Drone::virtualService(const Sensor& target, 
	double& prize, double& engy_cost) const
{
	prize = target.prize();
	engy_cost = target.chrgCost();
}

void Drone::virtualVist(const Point3D& dest, const double horz_dist, 
	const AvgPower& avg_powers, double& engy_cost) const
{
	std::unique_ptr<Point3D> temp_uav_coord = std::make_unique<Point3D>(_coord);
	double takeoff_engy = DS::getTakeoffEnergy(avg_powers.avg_p_takeoff);
	temp_uav_coord->z += DS::FLIGHT_ALT;
	double cruise_engy = DS::getCruiseEnergy(horz_dist, avg_powers.avg_p_cruise);
	double landing_engy = DS::getLandingEnergy(
		temp_uav_coord->distZAbs(dest), avg_powers.avg_p_landing);
	engy_cost = takeoff_engy + cruise_engy + landing_engy;
}

