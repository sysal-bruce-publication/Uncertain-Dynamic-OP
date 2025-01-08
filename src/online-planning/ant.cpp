#include "ant.h"

void Ant::reset(const DroneState& uav0, const std::vector<bool>& visit_mask)
{
	cur_node = _start_idx;
	fit = Fitness();
	_feasible = true;
	size_t num_nodes = _visit_mask.size();
	if (!path.empty()) { path.clear(); } path.reserve(num_nodes);
	path.push_back(_start_idx);
	_visit_mask = visit_mask;
	_uav = std::make_shared<Drone>(uav0);
}

void Ant::addSensorToPath(const size_t sensor_idx, const Sensor& sn,
	const double rth_engy, const double visit_engy)
{
	_uav->updateState(sn.coord(), visit_engy);
	double srvc_prize = 0, srvc_engy = 0;
	_uav->virtualService(sn, srvc_prize, srvc_engy);
	_uav->updateState(srvc_engy);
	fit.prize += srvc_prize;
	fit.cost += visit_engy + srvc_engy;

	path.push_back(sensor_idx);
	cur_node = sensor_idx;
	_visit_mask[sensor_idx] = true;
	
}

void Ant::addLastToPath(const double rth_engy)
{
	fit.cost += rth_engy;
	path.push_back(_end_idx);
	cur_node = _end_idx;
}

