#include "drone.h"

namespace DS = DroneSpec;
namespace CU = CommonUtils;
namespace RU = RandomUtils;

bool Drone::service(const double eta_ipt, const double eta_chrg, Sensor& target)
{
	double prize = 0, engy_cost = 0, time_cost = 0;
	target.checkIPTCost(eta_ipt, eta_chrg, prize, engy_cost, time_cost);

	_engy -= engy_cost;
	_simu_t += time_cost;

	//! Note that here only add charged energy when service is successful. But 
	//! we will update energy and simulation time anyway because this information
	//! should be showed in both terminal and mission log to explain why mission
	//! failed.
	if (_engy > EPS) { _chrg_engy += prize; return true; }
	else { return false; }
}

void Drone::_takeoff(const Point3D& dest, const double dist,
	double& time_cost, double& engy_cost, double& avg_pwr) const
{
	engy_cost = 0;
	time_cost = dist / DS::TAKEOFF_VEL;  //! [s]
	size_t num_steps = static_cast<size_t>(time_cost / _simu_dt);

	for (size_t i = 0; i < num_steps; i++) {
		double power = RU::normalRand(_takeoff_mean, _takeoff_std);
		engy_cost += power * _simu_dt;
	}
	double t_remain = time_cost - _simu_dt * num_steps;
	if (t_remain > 0) {
		engy_cost += RU::normalRand(_takeoff_mean, _takeoff_std) * t_remain;
	}
	avg_pwr = engy_cost / time_cost; 
	engy_cost /= 1000.;
}

void Drone::_cruise(const Point3D& dest, const double dist,
	double& time_cost, double& engy_cost, vector<double>& avg_pwrs) const
{
	engy_cost = 0;
	time_cost = dist / DS::CRUISE_VEL; //! Time [s], speed [m/s], distance [m]
	size_t num_steps = static_cast<size_t>(time_cost / _simu_dt);
	double t_remain = time_cost - _simu_dt * num_steps;
	
	double sample_engy_ref = 0;
	for (size_t i = 0; i < num_steps; i++) {
		double power = RU::normalRand(_cruise_mean, _cruise_std);
		engy_cost += power * _simu_dt;  //! [J]
		if ((i + 1) % _sample_per_simu_dt == 0) {
			double avg_power = (engy_cost - sample_engy_ref) / _sample_duration;
			avg_pwrs.push_back(avg_power);
			sample_engy_ref = engy_cost;
		}
	}
	if (t_remain > 0) { 
		engy_cost += RU::normalRand(_cruise_mean, _cruise_std) * t_remain;
	}

	avg_pwrs.push_back(engy_cost / time_cost);
	engy_cost /= 1000.;
}

void Drone::_landing(const Point3D& dest, const double dist,
	double& time_cost, double& engy_cost, double& avg_pwr) const
{
	engy_cost = 0;
	time_cost = dist / DS::LANDING_VEL;  //! [s]
	size_t num_steps = static_cast<size_t>(time_cost / _simu_dt);
	double t_remain = time_cost - _simu_dt * num_steps;
	
	for (size_t i = 0; i < num_steps; i++) {
		double power = RU::normalRand(_landing_mean, _landing_std);
		engy_cost += power * _simu_dt;
	}
	if (t_remain > 0) {
		engy_cost += RU::normalRand(_landing_mean, _landing_std) * t_remain;
	}
	avg_pwr = engy_cost / time_cost;
	engy_cost /= 1000.;
}

void Drone::virtualVisit(const Point3D& dest, const double horz_dist,
	double& visit_time, double& visit_engy, double& avg_pwr_t, 
	double& avg_pwr_l, vector<double>& avg_pwr_c) const
{
	std::unique_ptr<Point3D> temp_uav_coord = std::make_unique<Point3D>(_coord);
	std::unique_ptr<Point3D> temp_dest = std::make_unique<Point3D>(
		_coord.x, _coord.y, _coord.z + DS::FLIGHT_ALT);
	double takeoff_time = 0, takeoff_engy = 0;
	double takeoff_dist = temp_uav_coord->distZ(*temp_dest);
	_takeoff(*temp_dest, takeoff_dist, takeoff_time, takeoff_engy, avg_pwr_t);
	temp_uav_coord->z = temp_dest->z;
	temp_dest->x = dest.x; temp_dest->y = dest.y;
	double cruise_time = 0, cruise_engy = 0;
	double cruise_dist = temp_uav_coord->distXY(*temp_dest);
	_cruise(*temp_dest, cruise_dist, cruise_time, cruise_engy, avg_pwr_c);
	double landing_time = 0, landing_engy = 0;
	double landing_dist = temp_uav_coord->distZ(dest);
	_landing(dest, landing_dist, landing_time, landing_engy, avg_pwr_l);
	visit_time = takeoff_time + cruise_time + landing_time;
	visit_engy = takeoff_engy + cruise_engy + landing_engy;
}

void Drone::_takeoffInstPwr(const Point3D& dest, const double dist,
	double& time_cost, double& engy_cost, vector<double>& pwrs) const
{
	engy_cost = 0;
	time_cost = dist / DS::TAKEOFF_VEL;  //! [s]
	size_t num_steps = static_cast<size_t>(time_cost / _simu_dt);
	if (!pwrs.empty()) { pwrs.clear(); } pwrs.reserve(num_steps + 1);

	for (size_t i = 0; i < num_steps; i++) {
		double power = RU::normalRand(_takeoff_mean, _takeoff_std);
		pwrs.push_back(power);
		engy_cost += power * _simu_dt;
	}
	double t_remain = time_cost - _simu_dt * num_steps;
	if (t_remain > 0) {
		double power = RU::normalRand(_takeoff_mean, _takeoff_std);
		engy_cost += power * t_remain;
		pwrs.push_back(power);
	}
	engy_cost /= 1000.;
}

void Drone::_cruiseInstPwr(const Point3D& dest, const double dist,
	double& time_cost, double& engy_cost, vector<double>& pwrs) const
{
	engy_cost = 0;
	time_cost = dist / DS::CRUISE_VEL; //! Time [s], speed [m/s], distance [m]
	size_t num_steps = static_cast<size_t>(time_cost / _simu_dt);
	double t_remain = time_cost - _simu_dt * num_steps;
	if (!pwrs.empty()) { pwrs.clear(); } pwrs.reserve(num_steps + 1);

	for (size_t i = 0; i < num_steps; i++) {
		double power = RU::normalRand(_cruise_mean, _cruise_std);
		engy_cost += power * _simu_dt;  //! [J]
		pwrs.push_back(power);
	}
	if (t_remain > 0) {
		double power = RU::normalRand(_cruise_mean, _cruise_std);
		engy_cost += power * t_remain;
		pwrs.push_back(power);
	}
	engy_cost /= 1000.;
}

void Drone::_landingInstPwr(const Point3D& dest, const double dist,
	double& time_cost, double& engy_cost, vector<double>& pwrs) const
{
	engy_cost = 0;
	time_cost = dist / DS::LANDING_VEL;  //! [s]
	size_t num_steps = static_cast<size_t>(time_cost / _simu_dt);
	double t_remain = time_cost - _simu_dt * num_steps;
	if (!pwrs.empty()) { pwrs.clear(); } pwrs.reserve(num_steps + 1);

	for (size_t i = 0; i < num_steps; i++) {
		double power = RU::normalRand(_landing_mean, _landing_std);
		engy_cost += power * _simu_dt;
		pwrs.push_back(power);
	}
	if (t_remain > 0) {
		double power = RU::normalRand(_landing_mean, _landing_std);
		engy_cost += power * t_remain;
		pwrs.push_back(power);
	}
	engy_cost /= 1000.;
}

void Drone::virtualVisit(const Point3D& dest, const double horz_dist,
	double& visit_time, double& visit_engy, vector<double>& pwrs_minmaxs) const
{
	//! min and max for three actions
	if (!pwrs_minmaxs.empty()) { pwrs_minmaxs.clear(); } pwrs_minmaxs.reserve(6);
	std::unique_ptr<Point3D> temp_uav_coord = std::make_unique<Point3D>(_coord);
	std::unique_ptr<Point3D> temp_dest = std::make_unique<Point3D>(
		_coord.x, _coord.y, _coord.z + DS::FLIGHT_ALT);
	double takeoff_time = 0, takeoff_engy = 0;
	double takeoff_dist = temp_uav_coord->distZ(*temp_dest);
	vector<double> pwrs_t;
	_takeoffInstPwr(*temp_dest, takeoff_dist, takeoff_time, takeoff_engy, pwrs_t);
	temp_uav_coord->z = temp_dest->z;
	temp_dest->x = dest.x; temp_dest->y = dest.y;
	double cruise_time = 0, cruise_engy = 0;
	double cruise_dist = temp_uav_coord->distXY(*temp_dest);
	vector<double> pwrs_c;
	_cruiseInstPwr(*temp_dest, cruise_dist, cruise_time, cruise_engy, pwrs_c);
	double landing_time = 0, landing_engy = 0;
	double landing_dist = temp_uav_coord->distZ(dest);
	vector<double> pwrs_l;
	_landingInstPwr(dest, landing_dist, landing_time, landing_engy, pwrs_l);
	visit_time = takeoff_time + cruise_time + landing_time;
	visit_engy = takeoff_engy + cruise_engy + landing_engy;

	pwrs_minmaxs.push_back(CU::vecMinElem(pwrs_t));
	pwrs_minmaxs.push_back(CU::vecMinElem(pwrs_c));
	pwrs_minmaxs.push_back(CU::vecMinElem(pwrs_l));
	pwrs_minmaxs.push_back(CU::vecMaxElem(pwrs_t));
	pwrs_minmaxs.push_back(CU::vecMaxElem(pwrs_c));
	pwrs_minmaxs.push_back(CU::vecMaxElem(pwrs_l));
}