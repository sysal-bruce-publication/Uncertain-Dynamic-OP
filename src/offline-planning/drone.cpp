#include "drone.h"

using std::unique_ptr, std::make_unique;

namespace DS = DroneSpec;
namespace RU = RandomUtils;

bool Drone::service(const Sensor& target, double& prize, double& engy_cost) const
{
	prize = target.prize();
	engy_cost = target.chrgCost();

	if (_engy <= engy_cost + 1e-4) { return false; }
	return true;
}

bool Drone::_takeoffNoWind(const double dist, 
	double& time_cost, double& engy_cost) const
{
	time_cost = dist / DS::TAKEOFF_VEL;  //! [s]
	size_t num_steps = static_cast<size_t>(time_cost / _dt);

	engy_cost = 0;
	for (size_t t = 0; t < num_steps; ++t) {
		double power_now = DS::avgPwrTakeoff();
		engy_cost += power_now * _dt;  //! [J]
	}
	// Check if the last step is evenly divided
	double t_remain = time_cost - _dt * num_steps;
	if (t_remain > 1e-4) {  // If not evenly divided
		double power_now = DS::avgPwrTakeoff();
		engy_cost += power_now * t_remain;  //! [J]
	}
	engy_cost /= 1000.;  // Energy convert to [kJ]

	if (_engy <= engy_cost + 1e-4) { return false; }
	return true;
}

bool Drone::_cruiseNoWind(const double dist, 
	double& time_cost, double& engy_cost) const
{
	time_cost = dist / DS::CRUISE_VEL;  // Time [s], speed [m/s], distance [m]
	size_t num_steps = static_cast<size_t>(time_cost / _dt);
	engy_cost = 0;

	for (size_t t = 0; t < num_steps; ++t) {
		double power_now = DS::avgPwrCruise();
		engy_cost += power_now * _dt;  //! [J]
	}
	// Check if the last step is evenly divided
	double t_remain = time_cost - _dt * num_steps;
	if (t_remain > 1e-4) {  // If not evenly divided
		double power_now = DS::avgPwrCruise();
		engy_cost += power_now * t_remain;  //! [J]
	}
	engy_cost /= 1000.;  // Energy convert to [kJ]

	if (_engy <= engy_cost + 1e-4) { return false; }
	return true;
}

bool Drone::_landingNoWind(const double dist, 
	double& time_cost, double& engy_cost) const
{
	time_cost = dist / DS::LANDING_VEL;  //! [s]
	size_t num_steps = static_cast<size_t>(time_cost / _dt);

	engy_cost = 0;
	for (size_t t = 0; t < num_steps; ++t) {
		double power_now = DS::avgPwrLanding();
		engy_cost += power_now * _dt;  //! [J]
	}
	// Check if the last step is evenly divided
	double t_remain = time_cost - _dt * num_steps;
	if (t_remain > 1e-4) {  // If not evenly divided
		double power_now = DS::avgPwrLanding();
		engy_cost += power_now * t_remain;  //! [J]
	}
	engy_cost /= 1000.;  // Energy convert to [kJ]

	if (_engy <= engy_cost + 1e-4) { return false; }
	return true;
}

bool Drone::visitNoWind(const Point3D& dest, const double horz_dist, 
	double& time_cost, double& engy_cost) const
{
	time_cost =	engy_cost = 0;
	unique_ptr<Point3D> temp_uav_coord = make_unique<Point3D>(_coord);

	double takeoff_time = 0, takeoff_engy = 0;
	if (_takeoffNoWind(DS::FLIGHT_ALT, takeoff_time, takeoff_engy)) {
		time_cost += takeoff_time; engy_cost += takeoff_engy;
		temp_uav_coord->z += DS::FLIGHT_ALT;
	}
	else { return false; }

	double cruise_time = 0, cruise_engy = 0;
	double cruise_dist = horz_dist > 0 ? horz_dist : temp_uav_coord->distXY(dest);
	//! Here no need to update temp_uav_coord and don't need temp_dest any more.
	if (_cruiseNoWind(cruise_dist, cruise_time, cruise_engy)) {
		time_cost += cruise_time; engy_cost += cruise_engy;
	}
	else { return false; }

	double landing_time = 0, landing_engy = 0;
	double landing_dist = temp_uav_coord->distZ(dest);
	if (_landingNoWind(landing_dist, landing_time, landing_engy)) {
		time_cost += landing_time; engy_cost += landing_engy;
	}
	else { return false; }

	return true;
}

bool Drone::_takeoffWind(const Point3D& dest, 
	const double dist, double& engy_cost) const
{
	double time_cost = dist / DS::TAKEOFF_VEL;  // Time [s]
	engy_cost = DS::avgPwrTakeoffWind() * time_cost / 1000.;  // Energy [kJ]

	if (_engy <= engy_cost + 1e-4) return false;
	return true;
}

Point2D Drone::_cruiseGroundSpeedlAngle(const Point3D& dest) const
{
	double angle = atan2(dest.y - _coord.y, dest.x - _coord.x);
	return Point2D(DS::CRUISE_VEL * cos(angle), DS::CRUISE_VEL * sin(angle));
}

bool Drone::_cruiseWind(const Point3D& dest, 
	const double dist, double& engy_cost) const
{
	double time_cost = dist / DS::CRUISE_VEL;  // Time [s], speed [m/s], distance [m]
	Point2D v_gnd_xy = _cruiseGroundSpeedlAngle(dest);
	size_t num_steps = static_cast<size_t>(time_cost / _dt);
	engy_cost = 0;

	for (size_t t = 0; t < num_steps; ++t) {
		double power_now = DS::avgPwrCruiseWind(_uni_wind, v_gnd_xy);
		engy_cost += power_now * _dt;  //! [J]
	}
	// Check if the last step is evenly divided
	double t_remain = time_cost - _dt * num_steps;
	if (t_remain > 1e-4) {  // If not evenly divided
		double power_now = DS::avgPwrCruiseWind(_uni_wind, v_gnd_xy);
		engy_cost += power_now * t_remain;  //! [J]
	}
	engy_cost /= 1000.;  // Energy convert to [kJ]

	if (_engy <= engy_cost + 1e-4) { return false; }
	return true;
}

bool Drone::_landingWind(const Point3D& dest,
	const double dist, double& engy_cost) const
{
	double time_cost = dist / DS::LANDING_VEL;  // Time [s]
	engy_cost = DS::avgPwrLandingWind() * time_cost / 1000.;  // Energy [kJ]

	if (_engy <= engy_cost + 1e-4) return false;
	return true;
}

bool Drone::visitWind(const Point3D& dest,
	const double horz_dist, double& engy_cost) const
{
	engy_cost = 0;
	unique_ptr<Point3D> temp_uav_coord = make_unique<Point3D>(_coord);
	unique_ptr<Point3D> temp_dest = make_unique<Point3D>(
		_coord.x, _coord.y, _coord.z + DS::FLIGHT_ALT);

	double takeoff_engy = 0, takeoff_dist = temp_uav_coord->distZ(*temp_dest);
	if (_takeoffWind(*temp_dest, takeoff_dist, takeoff_engy)) {
		engy_cost += takeoff_engy;
		temp_uav_coord->z = temp_dest->z;
		temp_dest->x = dest.x; temp_dest->y = dest.y;
	}
	else { return false; }

	double cruise_engy = 0;
	double cruise_dist = horz_dist > 0 ? horz_dist : temp_uav_coord->distXY(*temp_dest);
	//! Here no need to update temp_uav_coord and don't need temp_dest any more.
	if (_cruiseWind(*temp_dest, cruise_dist, cruise_engy)) {
		engy_cost += cruise_engy;
	}
	else { return false; }

	double landing_engy = 0, landing_dist = temp_uav_coord->distZ(dest);
	if (_landingWind(dest, landing_dist, landing_engy)) {
		engy_cost += landing_engy;
	}
	else { return false; }

	return true;
}

