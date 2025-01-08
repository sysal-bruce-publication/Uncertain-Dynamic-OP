/*****************************************************************//**
 * \file   drone.h
 * \brief  The simulation of a drone's flight process. The UAV only
 * has 3 actions: ASCENT, CRUISE, DESCENT. The energy model is based
 * on a regression model of DJI M100.
 *
 * \author Qiuchen Qian
 * \date   March 2024
 *********************************************************************/
#ifndef __DRONE_H__
#define __DRONE_H__

#include "sensor.h"

struct AvgPower
{
	AvgPower() = default;
	AvgPower(const double cdf_value_, const double avg_p_takeoff_,
		const double avg_p_cruise_, const double avg_p_landing_) :
		cdf_value(cdf_value_),
		avg_p_takeoff(avg_p_takeoff_),
		avg_p_cruise(avg_p_cruise_),
		avg_p_landing(avg_p_landing_) {}

	double cdf_value     = 0;
	double avg_p_takeoff = 0;
	double avg_p_cruise  = 0;
	double avg_p_landing = 0;
};

namespace DroneSpec
{
	constexpr double FLIGHT_ALT      = 30;		// Flight altitude [m].
	constexpr double CRUISE_VEL      = 10;		// Cruise ground speed [m/s].
	constexpr double TAKEOFF_VEL     = 3;		// Takeoff ground speed [m/s].
	constexpr double LANDING_VEL     = 2;		// Landing ground speed [m/s].
	constexpr double WEIGHT_UAV      = 3.93;	// UAV overall weight [kg].
	constexpr double AIR_DENSITY     = 1.225;	// Air density [kg/m^3].
	constexpr double B0_TAKEOFF_MEAN = 13.8;
	constexpr double B0_TAKEOFF_STD  = 18.9;
	constexpr double B1_TAKEOFF_MEAN = 80.4;
	constexpr double B1_TAKEOFF_STD  = 2.6;
	constexpr double B0_CRUISE_MEAN  = 16.8;
	constexpr double B0_CRUISE_STD   = 15.0;
	constexpr double B1_CRUISE_MEAN  = 68.9;
	constexpr double B1_CRUISE_STD   = 2.0;
	constexpr double B0_LANDING_MEAN = -24.3;
	constexpr double B0_LANDING_STD  = 12.5;
	constexpr double B1_LANDING_MEAN = 71.5;
	constexpr double B1_LANDING_STD  = 1.7;

	static double WEIGHT_AIR_TERM  = sqrt(pow(WEIGHT_UAV, 3) / AIR_DENSITY);
	static double PWR_TAKEOFF_MEAN = WEIGHT_AIR_TERM * B1_TAKEOFF_MEAN + B0_TAKEOFF_MEAN;
	static double PWR_CRUISE_MEAN  = WEIGHT_AIR_TERM * B1_CRUISE_MEAN + B0_CRUISE_MEAN;
	static double PWR_LANDING_MEAN = WEIGHT_AIR_TERM * B1_LANDING_MEAN + B0_LANDING_MEAN;
	static double PWR_TAKEOFF_VAR  = pow(WEIGHT_AIR_TERM * B1_TAKEOFF_STD, 2)
		+ pow(B0_TAKEOFF_STD, 2);
	static double PWR_CRUISE_VAR = pow(WEIGHT_AIR_TERM * B1_CRUISE_STD, 2)
		+ pow(B0_CRUISE_STD, 2);
	static double PWR_LANDING_VAR = pow(WEIGHT_AIR_TERM * B1_LANDING_STD, 2)
		+ pow(B0_LANDING_STD, 2);

	static double getTakeoffEnergy(const double avg_power_takeoff)
	{
		return avg_power_takeoff * FLIGHT_ALT / TAKEOFF_VEL / 1000.;
	}

	static double getCruiseEnergy(const double dist, const double avg_power_cruise)
	{
		return avg_power_cruise * dist / CRUISE_VEL / 1000.;
	}

	static double getLandingEnergy(const double dist, const double avg_power_landing)
	{
		return avg_power_landing * dist / LANDING_VEL / 1000.;
	}
}

struct DroneState
{
	DroneState() = default;
	DroneState(const double engy_, const Point3D& coord_) :
		engy(engy_), coord(coord_) {}

	double engy   = 0;
	Point3D coord = Point3D();
};

class Drone
{
public:
	Drone() = default;
	Drone(const DroneState& uav0) :
		_engy(uav0.engy),
		_coord(uav0.coord) {}
	~Drone() {}

	double engy() const { return _engy; }
	Point3D coord() const { return _coord; }

	/**
	 * Service a target sensor node, i.e. UAV performs IPT to recharge the
	 * bank of supercapacitor. This function won't update UAV state in place,
	 * it only estimates the energy cost.
	 *
	 * \param target
	 * \param prize
	 * \param engy_cost
	 * \return
	 */
	void virtualService(const Sensor& target, double& prize, double& engy_cost) const;

	void virtualVist(const Point3D& dest, const double horz_dist, 
		const AvgPower& avg_powers, double& engy_cost) const;

	/**
	 * Update UAV coordinate.
	 *
	 * \param coord
	 */
	void updateState(const Point3D& coord) { _coord = coord; }

	/**
	 * Reduce UAV residual energy and increase UAV mission time.
	 *
	 * \param engy_diff
	 * \param time_diff
	 */
	void updateState(const double engy_diff) { _engy -= engy_diff; }

	/**
	 * Update UAV coordinate, reduce energy and increase mission time.
	 *
	 * \param coord
	 * \param engy_diff
	 * \param time_diff
	 */
	void updateState(const Point3D& coord, const double engy_diff)
	{
		_coord = coord; _engy -= engy_diff;
	}

private:
	//! Drone related variables
	double _engy        = 0;	// Energy level [kJ].
	Point3D _coord      = Point3D();				// UAV coordinates [m].
	const double _sample_dt    = 10;				// Simulation time step [s].

	void _takeoff(const double dist, const double avg_power_takeoff, 
		double& engy_cost) const;
	void _cruise(const double dist, const double avg_power_cruise,
		double& engy_cost) const;
	void _landing(const double dist, const double avg_power_landing,
		double& engy_cost) const;
};

#endif // !__DRONE_H__

