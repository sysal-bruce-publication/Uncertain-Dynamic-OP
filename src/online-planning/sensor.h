#pragma once
/*****************************************************************//**
 * \file   sensor.h
 * \brief  The class for sensor node model.
 *
 * \author Qiuchen Qian
 * \date   April 2024
 *********************************************************************/
#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "point.h"

namespace SensorSpec
{
	// Energy model specs for supercapacitor bank
	constexpr double V_MAX = 42;		// Maximum voltage [V]
	constexpr double V_MIN = 20;		// Minimum voltage [V]
	constexpr double SUPCAP_C = 10;		// Capacitance [F]
	constexpr double E_MAX = 8.82;	// Maximum energy 0.5 * 10 * 42^2 / 1000 [kJ]
	constexpr double E_MIN = 2;		// Minimum energy 0.5 * 10 * 20^2 / 1000 [kJ] 
	constexpr double IPT_ETA = 0.4;	// Inductive power transfer efficiency

	/**
	 * Estimate sensor node's remaining energy based on its voltage.
	 *
	 * \param v Supercapacitor bank's voltage [V].
	 * \return The energy of supercapacitor bank [kJ].
	 */
	inline double energy(const double v) { return SUPCAP_C * pow(v, 2) / 2000.; }
}

class Sensor
{
public:
	Sensor() = default;
	Sensor(const double x, const double y, const double z) :
		_coord(Point3D(x, y, z)) {}
	Sensor(const double x, const double y, const  double z, const double volt0) :
		_coord(Point3D(x, y, z)),
		_prize(SensorSpec::E_MAX - SensorSpec::energy(volt0)),
		_chrg_cost(_prize / SensorSpec::IPT_ETA) {}
	~Sensor() {}

	double prize() const { return _prize; }
	double chrgCost() const { return _chrg_cost; }
	Point3D coord() const { return _coord; }
	bool isSameCoord(const Sensor& s) const { return _coord == s._coord; }
	double distXY(const Sensor& s) const
	{
		return std::hypot(_coord.x - s._coord.x, _coord.y - s._coord.y);
	}

private:
	const Point3D _coord = Point3D();	// Sensor's coordinate [m]
	double _prize = 0;					// Sensor's prize, if fully charged [kJ]
	double _chrg_cost = 0;				// UAV's service (IPT) energy cost [kJ]
};

#endif // !__SENSOR_H__
