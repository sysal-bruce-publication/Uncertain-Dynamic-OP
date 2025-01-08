#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "point.h"

namespace SensorSpec
{
	// Energy model specs for supercapacitor bank
	constexpr double V_MAX         = 42;	// Maximum voltage [V]
	constexpr double V_MIN         = 20;	// Minimum voltage [V]
	constexpr double SUPCAP_C      = 10;	// Capacitance [F]
	constexpr double E_MAX         = 8.82;	// Maximum energy 0.5 * 10 * 42^2 / 1000 [kJ]
	constexpr double E_MIN         = 2;		// Minimum energy 0.5 * 10 * 20^2 / 1000 [kJ] 
	constexpr double IPT_ETA       = 0.4;	// Inductive power transfer efficiency
	constexpr double CHRG_I        = 0.825;	// Charging current [A]
	constexpr double CC2SUPCAP_ETA = 0.9;	// Charging efficiency from CC/CV charger to supercap bank 

	// Linear energy decay model specs for regular sensors
	constexpr double E_TOT_SENSE = 2.19444e-6;	// Energy consumption of MCU and sensing [kJ/s]
	constexpr double E_AUDIO     = 2.6e-4;		// Energy consumption of audio sensing, lasting 10 seconds [kJ]
	//constexpr double P_SENSE = 3.96e-3;
	//constexpr double P_IDLE  = 1.32e-5;
	//constexpr double P_COMM  = 2.45e-3;
	//constexpr double T_SENSE = 2e-3;
	//constexpr double T_IDLE = 9.498;
	//constexpr double T_RESET = 1;

	/**
	 * Estimate sensor node's remaining energy based on its voltage.
	 *
	 * \param v Sensor node's voltage [V].
	 * \return The energy of supercap bank [kJ].
	 */
	static double energy(const double v) { return SUPCAP_C * pow(v, 2) / 2000.; }

	/**
	 * Estimate charging time needed based on sensor node's voltage.
	 *
	 * \param v Sensor node's voltage [V].
	 * \return
	 */
	static double chargeTime(const double v, const double eta_cc2supcap)
	{
		return SUPCAP_C * (V_MAX - v) / CHRG_I / eta_cc2supcap;
	}

	/**
	 * Estimate the energy consumption caused by MCU and sensor operations with
	 * fixed sensing rate. Ref: https://ieeexplore.ieee.org/abstract/document/9184913.
	 *
	 * \param t Time passed [s].
	 * \return The energy consumption [kJ].
	 */
	static double linearDecayEnergy(const double t) { return E_TOT_SENSE * t; }

	//static double possionDecayEnergy(const double t) {}
}

class Sensor
{
public:
	Sensor() = default;
	Sensor(const double x, const double y, const double z) :
		_coord(Point3D(x, y, z)) {}
	Sensor(const double x, const double y, const double z, const double volt) :
		_coord(Point3D(x, y, z)),
		_volt(volt),
		_engy(SensorSpec::energy(_volt)) {}
	~Sensor() {}

	double volt() const { return _volt; }
	Point3D coord() const { return _coord; }
	bool isClose(const Sensor& s) const { return _coord == s._coord; }
	double distXY(const Sensor& s) const
	{
		return std::hypot(_coord.x - s._coord.x, _coord.y - s._coord.y);
	}

	void linearEnergyDecay(const double t)
	{
		_engy -= SensorSpec::linearDecayEnergy(t);
		_volt = sqrt(2000. * _engy / SensorSpec::SUPCAP_C);
	}

	void eventDrivenEnergyDecay(const double t) {}

	/**
	 * It's important to know UAV's arrival time at the sensor node so
	 * that we can calculate the sensor's voltage based on linear decay model or
	 * Poisson process model.
	 *
	 * \param prize The prize of charging this sensor node.
	 * \param engy_cost The energy cost of charging this sensor node [kJ].
	 * \param time_cost The time cost of charging this sensor node [s].
	 */
	void checkIPTCost(const double eta_ipt, const double eta_chrg,
		double& prize, double& engy_cost, double& time_cost) const;

	void updateStatusAfterIPT()
	{
		_engy = SensorSpec::E_MAX;
		_volt = SensorSpec::V_MAX;
	}

private:
	const Point3D _coord = Point3D();	// Sensor coordinate [m]
	double _volt = 0;					// Supercapacitor voltage
	double _engy = 0;					// Supercapacitor energy [kJ]
};

#endif // !__SENSOR_H__