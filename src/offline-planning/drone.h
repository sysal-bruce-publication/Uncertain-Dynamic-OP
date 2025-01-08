/*****************************************************************//**
 * \file   drone.h
 * \brief  The simulation of a drone's flight process. The UAV only
 * has 3 actions: TAKEOFF, CRUISE, LANDING. The energy model is based
 * on a regression model of DJI M100. 
 * 
 * \author Qiuchen Qian
 * \date   March 2024
 *********************************************************************/
#ifndef __DRONE_H__
#define __DRONE_H__

#include "sensor.h"

namespace DroneSpec
{
	//! UAV frame + coil = 2283 g, battery = 600 g, computer + sensors = 1113 g.
	constexpr double WEIGHT_PAYLOAD = 0.25;		// Payload weight [kg] (inc.ed in 3.93kg).
	constexpr double WEIGHT_UAV     = 3.93;		// UAV overall weight [kg].
	constexpr double FLIGHT_ALT     = 30;		// Flight altitude [m].
	constexpr double AIR_DENSITY    = 1.225;	// Air density [kg/m^3].
	constexpr double CRUISE_VEL     = 10;		// Cruise ground speed [m/s].
	constexpr double TAKEOFF_VEL    = 3;		// Takeoff ground speed [m/s].
	constexpr double LANDING_VEL    = 2;		// Landing ground speed [m/s].

	//! Approximation of power consumption model. Regression model parameters come 
	//! from TASE paper "Autonomous Recharging and Flight Mission Planning for 
	//! Battery - Operated Autonomous Drones".
	constexpr double BETA1 = -1.526;
	constexpr double BETA2 = 3.934;
	constexpr double BETA3 = 0.968;
	constexpr double BETA4 = 18.125;
	constexpr double BETA5 = 96.613;
	constexpr double BETA6 = -1.085;
	constexpr double BETA7 = 0.220;
	constexpr double BETA8 = 1.332;
	constexpr double BETA9 = 433.9;

	//! Linear regression model 
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

	static double WEIGHT_AIR_TERM = sqrt(pow(WEIGHT_UAV, 3) / AIR_DENSITY);
	static double PWR_CRUISE_MEAN = WEIGHT_AIR_TERM * B1_CRUISE_MEAN + B0_CRUISE_MEAN;
	static double PWR_CRUISE_VAR = pow(WEIGHT_AIR_TERM * B1_CRUISE_STD, 2)
		+ pow(B0_CRUISE_STD, 2);

	/**
	 * Calculate average power during TAKEOFF based on a regression model of M100.
	 * Ref: https://www.sciencedirect.com/science/article/pii/S2666389922001805.
	 *
	 * \return Power consumption [W].
	 */
	static double avgPwrTakeoff()
	{
		double b1 = RandomUtils::normalRand(B1_TAKEOFF_MEAN, B1_TAKEOFF_STD);
		double b0 = RandomUtils::normalRand(B0_TAKEOFF_MEAN, B0_TAKEOFF_STD);
		return b1 * WEIGHT_AIR_TERM + b0;
	}

	/**
	 * Calculate average power during TAKEOFF based on a regression model of M100.
	 * Ref: https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=9779119.
	 * 
	 * \return Power consumption [W].
	 */
	static double avgPwrTakeoffWind()
	{
		return BETA4 * TAKEOFF_VEL + BETA7 * WEIGHT_PAYLOAD + BETA9;
	}

	static double avgPwrCruise()
	{
		double b1 = RandomUtils::normalRand(B1_CRUISE_MEAN, B1_CRUISE_STD);
		double b0 = RandomUtils::normalRand(B0_CRUISE_MEAN, B0_CRUISE_STD);
		return b1 * WEIGHT_AIR_TERM + b0;
	}

	static double avgPwrCruiseWind(const Point2D& v_gnd_xy, const Point2D& v_wind_xy)
	{
		return BETA1 * CRUISE_VEL + BETA7 * WEIGHT_PAYLOAD
			+ BETA8 * v_gnd_xy.innerProd(v_wind_xy) + BETA9;
	}

	static double avgPwrLanding()
	{
		double b1 = RandomUtils::normalRand(B1_TAKEOFF_MEAN, B1_TAKEOFF_STD);
		double b0 = RandomUtils::normalRand(B0_TAKEOFF_MEAN, B0_TAKEOFF_STD);
		return b1 * WEIGHT_AIR_TERM + b0;
	}

	static double avgPwrLandingWind()
	{
		return BETA4* LANDING_VEL + BETA7 * WEIGHT_PAYLOAD + BETA9;
	}
}

struct DroneState
{
	DroneState() = default;
	DroneState(const double engy_, const double simu_dt_, const Point3D& coord_) :
		engy(engy_),
		dt(simu_dt_),
	    coord(coord_) {}

	double engy   = 0;
	double dt     = 0;
	Point3D coord = Point3D();
};

class Drone
{
public:
	Drone() = default;
	Drone(const DroneState& uav_state) :
		_dt(uav_state.dt),
		_engy(uav_state.engy),
		_coord(uav_state.coord) {}
	Drone(const DroneState& uav_state, const Point2D& uni_wind) :
		_dt(uav_state.dt),
		_engy(uav_state.engy),
		_coord(uav_state.coord),
		_uni_wind(uni_wind) {}
	~Drone() {}

	double dt() const { return _dt; }
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
	bool service(const Sensor& target, double& prize, double& engy_cost) const;

	/**
	 * Set a destination coordinate and visit, this process includes 3 actions:
	 * ASCENT -> CRUISE -> DESCENT. This function won't update UAV state in place,
	 * it only estimates the energy and time cost.
	 * 
	 * \param dest The coordinate of the sensor node, we assume the sensor node is 
	 *	buried 30 cm underground.
	 * \param horz_dist The cruise distance.
	 * \param engy_cost Estimated energy cost based a regression model.
	 * \return 
	 */
	bool visitNoWind(const Point3D& dest, const double horz_dist, 
		double& time_cost, double& engy_cost) const;

	bool visitWind(const Point3D& dest,
		const double horz_dist, double& engy_cost) const;

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
	double _engy            = 0;			// Energy level [kJ].
	Point3D _coord          = Point3D();	// UAV coordinates [m].
	const double _dt        = 10;			// Simulation time step [s].
	const Point2D _uni_wind = Point2D();		// Uniform Wind in XY [m/s].

	bool _takeoffNoWind(const double dist, 
		double& time_cost, double& engy_cost) const;
	bool _cruiseNoWind(const double dist, 
		double& time_cost, double& engy_cost) const;
	bool _landingNoWind(const double dist, 
		double& time_cost, double& engy_cost) const;

	bool _takeoffWind(const Point3D& dest,
		const double dist, double& engy_cost) const;
	Point2D _cruiseGroundSpeedlAngle(const Point3D& dest) const;
	bool _cruiseWind(const Point3D& dest,
		const double dist, double& engy_cost) const;
	bool _landingWind(const Point3D& dest,
		const double dist, double& engy_cost) const;
};

#endif // !__DRONE_H__

