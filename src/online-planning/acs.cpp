#include "acs.h"

using std::multimap;
using std::vector;
using std::unique_ptr, std::make_unique;
using std::pair, std::make_pair;
using std::tuple, std::make_tuple, std::get;
namespace RU = RandomUtils;
namespace CU = CommonUtils;

void ACS::checkSol(const Ant& ant_k) const
{
	double path_c = 0, path_p = 0;
	for (size_t i = 0; i < ant_k.path.size() - 1; i++) {
		path_c += (_edge_engy_costs[ant_k.path[i]][ant_k.path[i + 1]]
			+ _sensors[ant_k.path[i]].chrgCost());
		path_p += _sensors[ant_k.path[i]].prize();
	}
	if (abs(ant_k.fit.cost - path_c) > EPS) { throw std::runtime_error("Cost not same"); }
	if (abs(ant_k.fit.prize - path_p) > EPS) { throw std::runtime_error("Prize not same"); }
}

void ACS::initColony(vector<Sensor>& sensors, vector<size_t>& visit_hist,
	vector<vector<double>>& edge_engy_costs)
{
	_sensors = std::move(sensors);
	_num_nodes = _sensors.size();
	_visit_mask0 = vector<bool>(_num_nodes, false);
	size_t num_visited = visit_hist.size();
	for (size_t idx : visit_hist) { _visit_mask0[idx] = true; }
	_edge_engy_costs = std::move(edge_engy_costs);
	//! Now exclude END_IDX, because otherwise in further solution construction, 
	//! END_IDX may be added in advance.
	_visit_mask0[END_IDX] = true;

	_ants.reserve(_num_ants);
	for (size_t k = 0; k < _num_ants; ++k) {
		_ants.push_back(Ant(_start_idx, END_IDX, _uav0, _visit_mask0));
	}
	_gb_ant = make_unique<GBAnt>();
}

void ACS::updateNeighbEdgeCostsWeightedAvg(const size_t prev_idx,
	const double prev_visit_engy, const double act_weight, const double exp_weight)
{
	double err = (prev_visit_engy - _edge_engy_costs[prev_idx][_start_idx])
		/ _edge_engy_costs[prev_idx][_start_idx];
	double weight_err = act_weight * (err + 1) + exp_weight;

	for (size_t j = 1; j < _num_nodes; j++) {
		if (!_visit_mask0[j] || j == END_IDX) {
			_edge_engy_costs[_start_idx][j] *= weight_err;
		}
	}
	for (size_t i = _num_depots; i < _num_nodes; ++i) {
		if (!_visit_mask0[i]) {
			for (size_t j = 1; j < _num_nodes; ++j) {
				if ((!_visit_mask0[j] && i != j) || j == END_IDX) {
					_edge_engy_costs[i][j] *= weight_err;
				}
			}
		}
	}
}

void ACS::updatePosteriorDistributions(const double alpha_0, const double kappa_0,
	const vector<double>& samples_tof, const vector<double>& samples_crs,
	const vector<double>& samples_ldg)
{
	namespace DS = DroneSpec;
	_lst_tof = make_unique<LST>(
		alpha_0, kappa_0, DS::PWR_TAKEOFF_MEAN, DS::PWR_TAKEOFF_VAR);
	_lst_tof->inference(samples_tof);
	_lst_crs = make_unique<LST>(
		alpha_0, kappa_0, DS::PWR_CRUISE_MEAN, DS::PWR_CRUISE_VAR);
	_lst_crs->inference(samples_crs);
	_lst_ldg = make_unique<LST>(
		alpha_0, kappa_0, DS::PWR_LANDING_MEAN, DS::PWR_LANDING_VAR);
	_lst_ldg->inference(samples_ldg);
}

void ACS::getUpdatedAvgPowers(const vector<double>& cdf_probs,
	vector<AvgPower>& avg_powers) const
{
	for (const double cdf_prob : cdf_probs) {
		double pwr_tof = _lst_tof->getXValueWithCDF(cdf_prob);
		double pwr_crs = _lst_crs->getXValueWithCDF(cdf_prob);
		double pwr_ldg = _lst_ldg->getXValueWithCDF(cdf_prob);
		avg_powers.push_back(AvgPower(cdf_prob, pwr_tof, pwr_crs, pwr_ldg));
	}
}

void ACS::updateNeighbEdgeCostsBayesian(const AvgPower& avg_pwr)
{
	unique_ptr<Drone> bay_uav = make_unique<Drone>(_uav0);
	for (size_t j = 1; j < _num_nodes; j++) {
		if (!_visit_mask0[j] || j == END_IDX) {
			Point3D dest = _sensors[j].coord();
			double dist = bay_uav->coord().distXY(dest);
			bay_uav->virtualVist(dest, dist, avg_pwr, _edge_engy_costs[_start_idx][j]);
		}
	}
	for (size_t i = _num_depots; i < _num_nodes; ++i) {
		if (!_visit_mask0[i]) {
			for (size_t j = 1; j < _num_nodes; ++j) {
				if ((!_visit_mask0[j] && i != j) || j == END_IDX) {
					bay_uav->updateState(_sensors[i].coord());
					Point3D dest = _sensors[j].coord();
					double dist = bay_uav->coord().distXY(dest);
					bay_uav->virtualVist(dest, dist, avg_pwr, _edge_engy_costs[i][j]);
				}
			}
		}
	}
}

void ACS::getUpdatedNeighbEdgeCostsBayesian(const AvgPower& avg_pwr,
	vector<vector<double>>& new_costs) const
{
	new_costs = _edge_engy_costs;
	unique_ptr<Drone> bay_uav = make_unique<Drone>(_uav0);
	for (size_t j = 1; j < _num_nodes; j++) {
		Point3D dest = _sensors[j].coord();
		double dist = bay_uav->coord().distXY(dest);
		bay_uav->virtualVist(dest, dist, avg_pwr, new_costs[_start_idx][j]);
	}
	for (size_t i = _num_depots; i < _num_nodes; ++i) {
		if (!_visit_mask0[i]) {
			for (size_t j = 1; j < _num_nodes; ++j) {
				if ((!_visit_mask0[j] && i != j) || j == END_IDX) {
					bay_uav->updateState(_sensors[i].coord());
					Point3D dest = _sensors[j].coord();
					double dist = bay_uav->coord().distXY(dest);
					bay_uav->virtualVist(dest, dist, avg_pwr, new_costs[i][j]);
				}
			}
		}
	}
}

void ACS::updateNeighbEdgeCostsMCGreedy(const double p_t_len, const double p_c_len, 
	const double p_l_len, const double p_t_min, const double p_c_min, 
	const double p_l_min)
{
	double level = RU::uniformRand01();
	double pwr_t = p_t_min + level * p_t_len;
	double pwr_c = p_c_min + level * p_c_len;
	double pwr_l = p_l_min + level * p_l_len;
	unique_ptr<AvgPower> avg_pwr = make_unique<AvgPower>(0, pwr_t, pwr_c, pwr_l);
	unique_ptr<Drone> mcg_uav = make_unique<Drone>(_uav0);

	for (size_t j = 1; j < _num_nodes; j++) {
		if (!_visit_mask0[j] || j == END_IDX) {
			Point3D dest = _sensors[j].coord();
			double dist = mcg_uav->coord().distXY(dest);
			mcg_uav->virtualVist(
				dest, dist, *avg_pwr, _edge_engy_costs[_start_idx][j]);
		}
	}
	for (size_t i = _num_depots; i < _num_nodes; ++i) {
		if (!_visit_mask0[i]) {
			for (size_t j = 1; j < _num_nodes; ++j) {
				if ((!_visit_mask0[j] && i != j) || j == END_IDX) {
					mcg_uav->updateState(_sensors[i].coord());
					Point3D dest = _sensors[j].coord();
					double dist = mcg_uav->coord().distXY(dest);
					mcg_uav->virtualVist(
						dest, dist, *avg_pwr, _edge_engy_costs[i][j]);
				}
			}
		}
	}
}

bool ACS::isBudgetEnough(const double base_cost) const
{
	if (_uav0.engy > base_cost + 30) { return true; }
	else { return false; }
}

double ACS::_getDropCost(const size_t adj0, const size_t adj1,
	const size_t node2drop) const
{
	return _edge_engy_costs[adj0][node2drop] + _edge_engy_costs[node2drop][adj1]
		+ _sensors[node2drop].chrgCost() - _edge_engy_costs[adj0][adj1];
}

void ACS::_dropNodesUntilPathFeasible(Ant& ant_k)
{
	while (!ant_k.isPathFeasible() && ant_k.path.size() > 2) {
		//! drop value, {node, cost difference}.
		multimap<double, IdxVal> dropval_node_cost;
		//! Then calculate the initial dropval_node_cost.
		for (size_t j = 1; j < ant_k.path.size() - 1; ++j) {
			size_t cur_node = ant_k.path[j];
			size_t prev_node = ant_k.path[j - 1];
			size_t next_node = ant_k.path[j + 1];
			double sn_p = _sensors[cur_node].prize();
			double cost_diff = _getDropCost(prev_node, next_node, cur_node);
			dropval_node_cost.insert(
				make_pair(sn_p / cost_diff, make_pair(cur_node, cost_diff)));
		}
		size_t node2drop = dropval_node_cost.begin()->second.first;
		size_t idx2drop = CommonUtils::findIndexByElem(ant_k.path, node2drop);
		// This is for the first drop value map update
		ant_k.fit.prize -= _sensors[node2drop].prize();
		ant_k.fit.cost -= dropval_node_cost.begin()->second.second;
		// Before erase the index, record its previous and next neighbor node.
		ant_k.path.erase(ant_k.path.begin() + idx2drop);
		ant_k.unmaskVisit(node2drop);
		ant_k.updateFeasibility();
	}
}

double ACS::_getAddCost(const size_t adj0, const size_t adj1,
	const size_t node2add) const
{
	return _edge_engy_costs[adj0][node2add] + _edge_engy_costs[node2add][adj1]
		+ _sensors[node2add].chrgCost() - _edge_engy_costs[adj0][adj1];
}

multimap<double, IdxIdxVal, std::greater<double>> ACS::_getInitAddValMap(
	const Ant& ant_k) const
{
	typedef tuple<size_t, size_t, size_t, size_t> QuadIdx;
	multimap<double, IdxIdxVal, std::greater<double>> addval_node_cost;
	for (size_t v = _num_depots; v < _num_nodes; ++v) {
		if (!ant_k.isNodeVisited(v)) {
			// Distance, {node, node index in path}
			multimap<double, DualIdx> cost_nbrs;
			// Find neighbor nodes of node, here we don't consider the 
			// charging cost because here is to find the closest neighbor.
			for (size_t j = 0; j < ant_k.path.size(); j++) {
				cost_nbrs.insert(make_pair(_edge_engy_costs[ant_k.path[j]][v],
					make_pair(ant_k.path[j], j)));
			}
			// {node, node index in path}
			vector<DualIdx> nbr_pairs; nbr_pairs.reserve(3);
			for (auto it = cost_nbrs.begin(); it != cost_nbrs.end(); ++it) {
				nbr_pairs.push_back(make_pair(it->second.first, it->second.second));
				if (nbr_pairs.size() > 2) { cost_nbrs.clear();  break; }
			}
			// add cost, {nbr, adj, nbr index in path, adj index in path}
			multimap<double, QuadIdx> cost_nbr_pairs;
			// Now fill neighbor node pairs, their indices in path and addcost
			// Note that we DO NOT check whether picked neighbors are adjacent
			// because addval_map will be ITERATIVELY computed ANYWAY, if we 
			// directly return adjacent neighbor pair, it will raise a problem later.
			for (size_t i = 0; i < nbr_pairs.size(); ++i) {
				size_t nbr = nbr_pairs[i].first;
				size_t nbr_idx = nbr_pairs[i].second;
				// Here we DO NOT add pair of start and end node, i.e., 
				// IGNORE start_node -> end_node AND end_node -> start_node
				// because the first and last node in path 
				// MUST be at start and end node.
				if (nbr_idx < ant_k.path.size() - 1) {
					size_t next_idx = nbr_idx + 1;
					cost_nbr_pairs.insert(make_pair(
						_getAddCost(nbr, ant_k.path[next_idx], v), make_tuple(
							nbr, ant_k.path[next_idx], nbr_idx, next_idx)));
				}
				if (nbr_idx > 0) {
					size_t prev_idx = nbr_idx - 1;
					cost_nbr_pairs.insert(make_pair(
						_getAddCost(ant_k.path[prev_idx], nbr, v), make_tuple(
							ant_k.path[prev_idx], nbr, prev_idx, nbr_idx)));
				}
			}
			nbr_pairs.clear();  // We don't need it any more.
			// Get the smallest addcost of this node v.
			double addcost_v = 1e6; size_t idx2add_v = 0;
			// If any two nodes adjacent in path, then find the pair with
			// the SMALLEST addcost and find index to insert node v.
			addcost_v = cost_nbr_pairs.begin()->first;
			idx2add_v = std::max(  // left, node to add, right
				get<2>(cost_nbr_pairs.begin()->second),
				get<3>(cost_nbr_pairs.begin()->second));
			// If the addcost within budget constraint, add it into the addval_map
			if (addcost_v + ant_k.fit.cost < _uav0.engy - EPS) {
				addval_node_cost.insert(make_pair(_sensors[v].prize() / addcost_v,
					make_tuple(v, idx2add_v, addcost_v)));
			}
		}
	}
	return addval_node_cost;
}

void ACS::_iterativeAddNodes(Ant& ant_k, size_t& idx2add, size_t& add_node,
	multimap<double, IdxIdxVal, std::greater<double>>& addval_node_cost)
{
	// add_value, {node2add, add_idx, add_cost}
	vector<std::pair<double, IdxIdxVal>> map2update;
	map2update.reserve(addval_node_cost.size());
	while (add_node != DEPOT_IDX) { // true means there exists feasible nodes to add
		// ONLY update prize and cost FOR NOW
		ant_k.fit.prize += _sensors[add_node].prize();
		ant_k.fit.cost += get<2>(addval_node_cost.begin()->second);
		addval_node_cost.erase(addval_node_cost.begin());  // Erase added node
		// Update map2update
		CU::eraseIf(addval_node_cost,
			[this, add_node, idx2add, ant_k, &map2update](const auto& pair) -> bool
			{
				size_t unvis_node = get<0>(pair.second); // Get unvisited node
				size_t unvis_idx = get<1>(pair.second);  // Get index to insert
				double unvis_p = _sensors[unvis_node].prize();
				// Update this unvis_node's add_value
				multimap<double, IdxIdxVal, std::greater<double>> temp_map;
				if (unvis_idx != idx2add && unvis_idx != idx2add + 1) {
					// Only when the previous add node is NOT LOCATED ADJACENT to the 
					// current add node, then insert previous calculated addval_map.
					if (idx2add < unvis_idx) {
						temp_map.insert(make_pair(pair.first, make_tuple(
							unvis_node, unvis_idx + 1, get<2>(pair.second))));
					}
					else { temp_map.insert(pair); }
				}
				// Otherwise, there exists two scenarios:
				// 1. Previous idx2add is located before or after current unvis_idx;
				// 2. Insertion of previous add node will lead to shorter addcost.
				// BOTH scenario will need to compute the addcost of inserting 
				// unvis_node before and after add_node. 
				double addcost_before_node2add = _getAddCost(
					ant_k.path[idx2add - 1], add_node, unvis_node);
				temp_map.insert(make_pair(unvis_p / addcost_before_node2add,
					make_tuple(unvis_node, idx2add, addcost_before_node2add)));
				double addcost_aftr_node2add = _getAddCost(
					add_node, ant_k.path[idx2add], unvis_node);
				temp_map.insert(make_pair(unvis_p / addcost_aftr_node2add,
					make_tuple(unvis_node, idx2add + 1, addcost_aftr_node2add)));
				// If meets the constraint, add it into addval_node_pair_cost and 
				// erase the iterator (because the value MAY not be updated)
				if (get<2>(temp_map.begin()->second)
					+ ant_k.fit.cost < _uav0.engy - EPS) {
					map2update.push_back(*temp_map.begin());
				}
				// If the smallest addcost cannot meet the budget 
				// OR it has been updated, erase the iterator.
				return true;
			});
		// Insert back updated unvisited node
		for (const auto& pair : map2update) { addval_node_cost.insert(pair); }
		map2update.clear();  // Has done its job.
		map2update.reserve(addval_node_cost.size());
		// This is updating the PREVIOUS node2add !
		CU::vecInsertElem(ant_k.path, idx2add, add_node);
		ant_k.maskVisit(add_node);
		ant_k.updateFeasibility();
		// Then get the CURRENT node2add
		if ((!addval_node_cost.empty()) && addval_node_cost.begin()->first > 0) {
			add_node = get<0>(addval_node_cost.begin()->second);
			idx2add = get<1>(addval_node_cost.begin()->second);
		}
		else { add_node = DEPOT_IDX; }
	}
}

void ACS::_addNodesUntilBudget(Ant& ant_k)
{
	if ((!ant_k.isAllVisited()) && ant_k.isPathFeasible()) {
		size_t node2add = DEPOT_IDX, idx2add = DEPOT_IDX;
		// add_value, {node to add, path index to add, add_cost}
		multimap<double, IdxIdxVal, std::greater<double>> addval_node_cost =
			_getInitAddValMap(ant_k);
		if ((!addval_node_cost.empty()) && addval_node_cost.begin()->first > 0) {
			// Get the node v and index to add in current path (whose 
			// add value is positive and largest.
			node2add = get<0>(addval_node_cost.begin()->second);
			idx2add = get<1>(addval_node_cost.begin()->second);
		}
		else { node2add = DEPOT_IDX; }
		// Then iteratively add nodes until no room to add.
		_iterativeAddNodes(ant_k, idx2add, node2add, addval_node_cost);
	}
}

void ACS::_globalUpdatingRule()
{
	for (size_t i = 0; i < _num_nodes; i++) {
		if (!_visit_mask0[i]) {  //! END_IDX already excluded here.
			for (size_t j = 0; j < _num_nodes; j++) {
				if (!_visit_mask0[j] && i != j) { _taus[i][j] *= _evap_rate; }
			}
		}
	}
	for (size_t j = 0; j < _gb_ant->path.size() - 1; ++j) {
		_taus[_gb_ant->path[j]][_gb_ant->path[j + 1]] += _gb_ant->delta_tau;
	}
}

bool ACS::initPhmoneMatrix(vector<size_t>& last_path)
{
	//! Update last path's cost and prize 
	size_t act_last_path_len = last_path.size() - 1;
	unique_ptr<Ant> last_ant = make_unique<Ant>(
		_start_idx, END_IDX, _uav0, _visit_mask0);
	for (size_t i = 2; i < act_last_path_len; i++) {  //! Start idx already added.
		size_t sn_idx = last_path[i];
		last_ant->addSensorToPath(sn_idx, _sensors[sn_idx],
			_edge_engy_costs[sn_idx][END_IDX], _edge_engy_costs[last_ant->cur_node][sn_idx]);
	}
	last_ant->addLastToPath(_edge_engy_costs[last_ant->cur_node][END_IDX]);
	last_ant->updateFeasibility();
	_dropNodesUntilPathFeasible(*last_ant);
	_addNodesUntilBudget(*last_ant);

	_gb_ant->fit.cost = last_ant->fit.cost;
	_gb_ant->fit.prize = last_ant->fit.prize;
	_gb_ant->path = last_ant->path;

	if (last_ant->path.size() < 3) { return false; } //! No need to evolve.

	//! tau0 = prize / cost / num_edges
	_tau0 = last_ant->fit.prize / last_ant->fit.cost / (last_ant->path.size() - 1);
	_taus = vector<vector<double>>(_num_nodes, vector<double>(_num_nodes, _tau0));
	_gb_ant->delta_tau = _alpha * _gb_ant->fit.prize / _gb_ant->fit.cost;
	//! If we let inheritance as the 1st iteration, then no need to call 
	//! local updating rule because all tau[i][j] = tau0.
	// _globalUpdatingRule();
	return true;
}

size_t ACS::_stateTransitionRule(const Ant& ant_k) const
{
	size_t sensor_idx = DEPOT_IDX;
	multimap<double, size_t, std::greater<double>> prod_nodes_map;
	for (size_t v = _num_depots; v < _num_nodes; ++v) {
		if (!ant_k.isNodeVisited(v)) {
			double eta_term = pow(_sensors[v].prize() /
				(_edge_engy_costs[ant_k.cur_node][v] + _sensors[v].chrgCost()), _beta);
			prod_nodes_map.insert(make_pair(_taus[ant_k.cur_node][v] * eta_term, v));
		}
	}

	if (RU::uniformRand01() <= _q0) { sensor_idx = prod_nodes_map.begin()->second; }
	else {
		vector<double> prob_js; prob_js.resize(_num_nodes);
		for (auto it = prod_nodes_map.begin(); it != prod_nodes_map.end(); ++it) {
			prob_js[it->second] = it->first;
		}
		double neighb_sum = CU::vecElemsSum(prob_js);
		std::transform(prob_js.begin(), prob_js.end(), prob_js.begin(),
			[neighb_sum](double& c) -> double { return c / neighb_sum; });
		RU::vecBiasedSampleOneIndex(prob_js, prob_js, sensor_idx);
	}
	return sensor_idx;
}

void ACS::_localUpdatingRule(const size_t i, const size_t j)
{
	if (_taus[i][j] > _tau0) { _taus[i][j] -= _rho * (_taus[i][j] - _tau0); }
}

void ACS::_pathConstruction(Ant& ant_k)
{
	// Here we add sensors to the ant's path until all sensors are visited or
	// the path is infeasible.
	while ((!ant_k.isAllVisited()) && ant_k.isPathFeasible()) {
		size_t sensor_idx = _stateTransitionRule(ant_k);
		_localUpdatingRule(ant_k.cur_node, sensor_idx);
		ant_k.addSensorToPath(sensor_idx, _sensors[sensor_idx],
			_edge_engy_costs[sensor_idx][END_IDX],
			_edge_engy_costs[ant_k.cur_node][sensor_idx]);
		ant_k.updateFeasibility();
	}
	// Add last node to the ant's path.
	ant_k.addLastToPath(_edge_engy_costs[ant_k.cur_node][END_IDX]);
	ant_k.updateFeasibility();
}

void ACS::_2opt(Ant& ant_k)
{
	auto delta_cost = [this](const vector<size_t>& path,
		const size_t v1, const size_t v2) -> double
		{
			size_t v1_next = v1 + 1, v2_next = v2 + 1;
			double cost_2edge = (-_edge_engy_costs[path[v1]][path[v1_next]]
				+ _edge_engy_costs[path[v1]][path[v2]]
				- _edge_engy_costs[path[v2]][path[v2_next]]
				+ _edge_engy_costs[path[v1_next]][path[v2_next]]);

			double rev_cost = 0;
			if (v2 > v1_next) { //! Because the edge cost is asymmetric, add
				//! extra cost = (edge_cost[v1][v1_next] + ... + edge_cost[v2_prev][v2]
				//!          - edge_cost[v2][v2_prev] - ... - edge_cost[v1_next][v1])
				for (size_t i = v1_next; i < v2; i++) {
					rev_cost += (-_edge_engy_costs[path[i]][path[i + 1]]
						+ _edge_engy_costs[path[i + 1]][path[i]]);
				}
			}
			return cost_2edge + rev_cost;
		};

	bool improved = true;
	size_t cnt = 0;
	size_t path_len = ant_k.path.size();
	while (improved && cnt < _num_iters_2opt) {
		improved = false;
		for (size_t i = 0; i < path_len - 2; ++i) {
			for (size_t j = i + 2; j < path_len - 1; ++j) {
				//! j start from i + 2 because if j=i+1, then no swap 
				double cost_diff = delta_cost(ant_k.path, i, j);
				if (cost_diff < 0) {
					CU::vecReverse(ant_k.path, i, j);
					ant_k.fit.cost += cost_diff;
					improved = true;
					cnt++;
				}
			}
		}
	}
}

void ACS::_updateGlobalBestAnt()
{
	//! New prize should be higher than original prize.
	double prize_diff = _ants[_lb_ant_id].fit.prize - _gb_ant->fit.prize;
	if (prize_diff >= _tol_acs) {  // General ant higher prize
		_gb_ant->fit.cost = _ants[_lb_ant_id].fit.cost;
		_gb_ant->fit.prize = _ants[_lb_ant_id].fit.prize;
		_gb_ant->path = _ants[_lb_ant_id].path;
		_gb_ant->delta_tau = _alpha * _gb_ant->fit.prize / _gb_ant->fit.cost;
		_no_impr_cnt = 0; return;
	}
	else if (abs(prize_diff) <= EPS) {  //! OR new cost is less than original cost.
		double cost_diff = _gb_ant->fit.cost - _ants[_lb_ant_id].fit.cost;
		if (cost_diff >= _tol_acs) {  // General ant less cost
			_gb_ant->fit.cost = _ants[_lb_ant_id].fit.cost;
			_gb_ant->fit.prize = _ants[_lb_ant_id].fit.prize;
			_gb_ant->path = _ants[_lb_ant_id].path;
			_gb_ant->delta_tau = _alpha * _gb_ant->fit.prize / _gb_ant->fit.cost;
			_no_impr_cnt = 0; return;
		}
	}
	_no_impr_cnt++;
}

void ACS::resetColony()
{
	for (size_t k = 0; k < _num_ants; k++) { _ants[k].reset(_uav0, _visit_mask0); }
}

void ACS::evolveUntilStopCriteria()
{
	vector<int> valid_nodes; valid_nodes.reserve(_num_nodes - _num_depots);
	for (int i = static_cast<int>(_num_depots); i < static_cast<int>(_num_nodes); i++) {
		if (!_visit_mask0[i]) { valid_nodes.push_back(static_cast<int>(i)); }
	}
	valid_nodes.shrink_to_fit();
	size_t num_targets = valid_nodes.size();

	vector<int> start_nodes; start_nodes.reserve(_num_ants);
	for (size_t t = 0; t < _num_iters_acs; t++) {
		if (_no_impr_cnt > _max_no_impr) { break; }
		Fitness lb_fit = Fitness(1e8, -1);

		//! Uniformly sample start node for each ant.
		if (_num_ants > num_targets) {
			//! If same length, shuffle node indices.
			start_nodes.insert(
				start_nodes.end(), valid_nodes.begin(), valid_nodes.end());;
			for (size_t i = num_targets; i < _num_ants; i++) {
				int rand_idx = 0;
				RU::vecUniformRandSampleOneElem(valid_nodes, rand_idx);
				start_nodes.push_back(rand_idx);
			}
		}
		else { RU::vecUniformRandSample(valid_nodes, _num_ants, start_nodes); }

		for (size_t k = 0; k < _num_ants; ++k) {
			// Set the first random start node.
			size_t idx1 = static_cast<size_t>(start_nodes[k]);
			_ants[k].addSensorToPath(idx1, _sensors[idx1],
				_edge_engy_costs[idx1][END_IDX], _edge_engy_costs[_start_idx][idx1]);
			//! [PATH CONSTRUCTION] Construct the solution path
			_pathConstruction(_ants[k]);
			//! [2-OPT] Local search
			if (_num_iters_2opt > 0 && _ants[k].path.size() > 3) {
				_2opt(_ants[k]); _ants[k].updateFeasibility();
			}
			//! [DROP NODE] Keep dropping node until path is feasible.
			_dropNodesUntilPathFeasible(_ants[k]);
			//! [ADD NODE] Keep adding node until no unvisited nodes,
			//! reach budget constraint, or reach number of searches.
			_addNodesUntilBudget(_ants[k]);
			//! Determine the locally best ant for later updating globally best ant.
			if (_ants[k].fit > lb_fit) { lb_fit = _ants[k].fit; _lb_ant_id = k; }
		}
		_updateGlobalBestAnt();
		_globalUpdatingRule();
		resetColony();
	}
}

double ACS::getPrizeSum() const
{
	double prize_sum = 0;
	for (size_t i = 0; i < _sensors.size(); i++) { prize_sum += _sensors[i].prize(); }
	return prize_sum;
}