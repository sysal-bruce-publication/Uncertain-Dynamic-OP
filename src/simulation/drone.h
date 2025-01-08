#ifndef __DRONE_H__
#define __DRONE_H__

#include "sensor.h"

namespace DroneSpec
{
	constexpr double FLIGHT_ALT  = 30;			// Flight altitude [m].
	constexpr double CRUISE_VEL  = 10;			// Cruise ground speed [m/s].
	constexpr double TAKEOFF_VEL = 3;			// Takeoff ground speed [m/s].
	constexpr double LANDING_VEL = 2;			// Landing ground speed [m/s].
}

class Drone
{
public:
	Drone() = default;
	Drone(const double simu_dt, const size_t sample_per_simu_dt, 
		const double takeoff_mean, const double takeoff_std, 
		const double cruise_mean, const double cruise_std,
		const double landing_mean, const double landing_std):
		_simu_dt(simu_dt),
		_sample_per_simu_dt(sample_per_simu_dt),
		_sample_duration(sample_per_simu_dt * simu_dt),
		_takeoff_mean(takeoff_mean), _takeoff_std(takeoff_std),
		_cruise_mean(cruise_mean), _cruise_std(cruise_std),
		_landing_mean(landing_mean), _landing_std(landing_std) {}
	Drone(const double simu_dt, const size_t sample_per_simu_dt, const double simu_t0, 
		const double uav_engy0, const double chrg_engy0, const Point3D& coord):
		_simu_t(simu_t0),
		_simu_t0(simu_t0),
		_simu_dt(simu_dt),
		_sample_per_simu_dt(sample_per_simu_dt),
		_sample_duration(sample_per_simu_dt * simu_dt),
		_engy(uav_engy0),
		_engy0(uav_engy0),
		_chrg_engy(chrg_engy0),
		_chrg_engy0(chrg_engy0),
		_coord(coord) {}
	~Drone() {}

	double getEnergyNow() const { return _engy; }
	double getDeltaEnergy() const { return _engy0 - _engy; }
	double getTimeNow() const { return _simu_t; }
	double getDeltaTime() const { return _simu_t - _simu_t0; }
	double getChargedEnergy() const { return _chrg_engy; }
	double getDeltaChargedEnergy() const { return _chrg_engy - _chrg_engy0; }

	void setCoord(const Point3D& coord) { _coord = coord; }
	void setStatus(const double time_passed, const double engy_consumed) 
	{
		_simu_t += time_passed; _engy -= engy_consumed;
	}
	bool service(const double eta_ipt, const double eta_chrg, Sensor& target);
	void virtualVisit(const Point3D& dest, const double horz_dist,
		double& visit_time, double& visit_engy, double& avg_pwr_t, 
		double& avg_pwr_l, vector<double>& avg_pwr_c) const;
	void virtualVisit(const Point3D& dest, const double horz_dist,
		double& visit_time, double& visit_engy, vector<double>& pwrs_minmaxs) const;


private:
	double _engy                     = 0;
	double _chrg_engy                = 0;
	double _simu_t                   = 0;
	Point3D _coord                   = Point3D();
	const double _simu_t0            = 0;	// When this simulation starts [s].
	const double _simu_dt            = 0;	// Simulation time step [s].
	const size_t _sample_per_simu_dt = 0;   // Sample Energy per x simulation time step.
	const double _sample_duration    = 0;	// Average power over this duration.
	const double _engy0              = 0;	// Initial energy level [kJ].
	const double _chrg_engy0         = 0;  	 // Initial charged energy level [kJ].
	const double _engy_coeff_min     = 0;	// Coefficient for real energy consumption
	const double _engy_coeff_max     = 0;	// Coefficient for real energy consumption
	const double _takeoff_mean       = 0;
	const double _takeoff_std        = 0;
	const double _cruise_mean        = 0;
	const double _cruise_std         = 0;
	const double _landing_mean       = 0;
	const double _landing_std        = 0;

	bool _isSafe(const double engy_cost) const { return _engy > engy_cost + EPS; }
	void _takeoff(const Point3D& dest, const double dist,
		double& time_cost, double& engy_cost, double& avg_pwr) const;
	void _cruise(const Point3D& dest, const double dist,
		double& time_cost, double& engy_cost, vector<double>& avg_pwrs) const;
	void _landing(const Point3D& dest, const double dist,
		double& time_cost, double& engy_cost, double& avg_pwr) const;
	void _takeoffInstPwr(const Point3D& dest, const double dist,
		double& time_cost, double& engy_cost, vector<double>& avg_pwr) const;
	void _cruiseInstPwr(const Point3D& dest, const double dist,
		double& time_cost, double& engy_cost, vector<double>& pwrs) const;
	void _landingInstPwr(const Point3D& dest, const double dist,
		double& time_cost, double& engy_cost, vector<double>& avg_pwr) const;
};

#endif // !__DRONE_H__

