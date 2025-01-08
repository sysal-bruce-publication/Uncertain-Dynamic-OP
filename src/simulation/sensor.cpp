#include "sensor.h"

namespace SS = SensorSpec;

void Sensor::checkIPTCost(const double eta_ipt, const double eta_chrg,
	double& prize, double& engy_cost, double& time_cost) const
{
	prize = SS::E_MAX - _engy;			// Prize is charged energy [kJ]
	engy_cost = prize / eta_ipt;	// Energy cost at UAV battery [kJ]
	time_cost = SS::chargeTime(_volt, eta_chrg);
}

