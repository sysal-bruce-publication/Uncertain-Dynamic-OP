#include "ant.h"

void Ant::reset(const DroneState& uav0)
{
	cur_node = START_IDX;
	fit = Fitness();
	_feasible = true;
	size_t num_nodes = _visit_mask.size();
	if (!path.empty()) { path.clear(); } path.reserve(num_nodes); 
	path.push_back(START_IDX);
	std::fill(_visit_mask.begin(), _visit_mask.end(), false);
	_visit_mask[START_IDX] = _visit_mask[END_IDX] = true;
	_uav = std::make_shared<Drone>(uav0);
}

bool Ant::safeAddSensorToPath(const size_t sensor_idx, const Sensor& sn,
	const double rth_engy, const double visit_engy)
{
	double srvc_prize = 0, srvc_engy = 0;
	_uav->service(sn, srvc_prize, srvc_engy);
	if (_uav->engy() >= visit_engy + srvc_engy + rth_engy) {
		_uav->updateState(sn.coord(), visit_engy);
		_uav->updateState(srvc_engy);
		fit.prize += srvc_prize; 
		fit.cost += visit_engy + srvc_engy;
		path.push_back(sensor_idx);
		cur_node = sensor_idx;
		_visit_mask[sensor_idx] = true;
		_feasible = true;
		return true;
	}
	else { _feasible = false; return false; }
}

void Ant::addSensorToPath(const size_t sensor_idx, const Sensor& sn, 
	const double rth_engy, const double visit_engy)
{
	_uav->updateState(sn.coord(), visit_engy);
	double srvc_prize = 0, srvc_engy = 0;
	_uav->service(sn, srvc_prize, srvc_engy);
	_uav->updateState(srvc_engy);
	fit.prize += srvc_prize; 
	fit.cost += visit_engy + srvc_engy;

	path.push_back(sensor_idx);
	cur_node = sensor_idx;
	_visit_mask[sensor_idx] = true;
	_feasible = _uav->engy() >= rth_engy - EPS ? true : false;
}

void Ant::addLastToPath(const double rth_engy)
{
	fit.cost += rth_engy;
	path.push_back(END_IDX);
	cur_node = END_IDX;
	_feasible = _uav->engy() < 0 ? false : true;
}

