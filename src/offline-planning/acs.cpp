#include "acs.h"

using std::multimap;
using std::vector;
using std::unique_ptr, std::make_unique;
using std::pair, std::make_pair;
using std::tuple, std::make_tuple, std::get;
namespace RU = RandomUtils;

void ACS::checkPathCostPrize(const vector<size_t>& path,
	const double cost_ref, const double prize_ref) const
{
	double cost = 0, prize = 0;
	for (size_t i = 0; i < path.size() - 1; i++) {
		cost += _edge_costs[path[i]][path[i + 1]] + _sensors[path[i]].chrgCost();
		prize += _sensors[path[i]].prize();
	}

	if (abs(cost - cost_ref) > 1e-2) {
		std::cerr << "[ERROR] Exp. cost: " << cost << " alg. cost" << cost_ref << "\n";
		throw std::runtime_error("");
	}
	if (abs(prize - prize_ref) > 1e-2) {
		std::cerr << "[ERROR] Exp. prize: " << prize << " alg. prize" << prize_ref << "\n";
		throw std::runtime_error("");
	}
}

double ACS::getPathCostWithWindModel(const vector<size_t>& path, 
	const Point2D& uni_wind) const
{
	double cost_wind = 0;
	unique_ptr<Drone> temp_uav = make_unique<Drone>(_uav0, uni_wind);
	for (size_t i = 0; i < path.size() - 1; i++) {
		temp_uav->updateState(_sensors[path[i]].coord());
		double horz_dist = _sensors[path[i]].coord().distXY(
			_sensors[path[i + 1]].coord());
		double cost_engy = 0;
		temp_uav->visitWind(_sensors[path[i + 1]].coord(), horz_dist, cost_engy);
		cost_wind += cost_engy + _sensors[path[i]].chrgCost();
	}
	return cost_wind;
}

void ACS::initColony(vector<Sensor>& sensors)
{
	_sensors = std::move(sensors);
	_num_nodes = _sensors.size();
	_ants.reserve(_num_ants);
	for (size_t k = 0; k < _num_ants; ++k) { 
		_ants.push_back(Ant(_num_nodes, _uav0));
	}
	_gb_ant = make_unique<GBAnt>();
}

void ACS::_getInitEdgeCosts()
{
	// Calculate distance matrix first.
	vector<vector<double>> dists = vector<vector<double>>(
		_num_nodes, vector<double>(_num_nodes));
	for (size_t i = 0; i < _num_nodes - 1; i++) {
		for (size_t j = i + 1; j < _num_nodes; j++) {
			if (i != END_IDX) {
				dists[i][j] = dists[j][i] = _sensors[i].distXY(_sensors[j]);
			}
		}
	}
	// Then based on cruise distance, calculate UAV visit energy cost.
	unique_ptr<Drone> temp_uav = make_unique<Drone>(_uav0);
	_edge_costs = vector<vector<double>>(_num_nodes, vector<double>(_num_nodes));
	//! Note that the index of start depot is 0, end depot is 1.
	for (size_t i = 0; i < _num_nodes; ++i) {
		if (i == END_IDX) { continue; }
		for (size_t j = 1; j < _num_nodes; ++j) {
			//! No self-loop, no cost from end depot to any node include start depot.
			//! If similar coordinates, no cost.
			if (i != j) {
				double time_cost = 0, engy_cost = 0;
				temp_uav->updateState(_sensors[i].coord());
				temp_uav->visitNoWind(
					_sensors[j].coord(), dists[i][j], time_cost, engy_cost);
				_edge_costs[i][j] = engy_cost;
			}
		}
	}
}

double ACS::_nearNgbrPrizeCostRatio() const
{
	Ant nn_ant = Ant(_num_nodes, _uav0);
	multimap<double, size_t> next_costs;
	while (!nn_ant.isAllVisited()) {
		for (size_t i = _num_depots; i < _num_nodes; i++) {
			if (!nn_ant.isNodeVisited(i)) {
				next_costs.insert(make_pair(_edge_costs[nn_ant.cur_node][i], i));
			}
		}
		// UAV visit and service target sensor node.
		double visit_engy = next_costs.begin()->first;
		size_t sensor_idx = next_costs.begin()->second;
		next_costs.clear();
		// Then check if it can be added into path.
		if (!nn_ant.safeAddSensorToPath(sensor_idx, _sensors[sensor_idx],
			_edge_costs[sensor_idx][END_IDX], visit_engy)) { break; }
	}
	// Update RTH cost.
	nn_ant.addLastToPath(_edge_costs[nn_ant.cur_node][END_IDX]);
	return nn_ant.fit.prize / nn_ant.fit.cost / (nn_ant.path.size() - 1);
}

void ACS::initPhmoneMatrix()
{
	_getInitEdgeCosts();
	_tau0 = _nearNgbrPrizeCostRatio();
	_taus = vector<vector<double>>(_num_nodes, vector<double>(_num_nodes, _tau0));
}

size_t ACS::_stateTransitionRule(const Ant& ant_k) const
{
	size_t sensor_idx = START_IDX;
	multimap<double, size_t, std::greater<double>> prod_nodes_map;
	for (size_t v = _num_depots; v < _num_nodes; ++v) {
		if (!ant_k.isNodeVisited(v)) { 
			double eta_term = pow(_sensors[v].prize() / 
				(_edge_costs[ant_k.cur_node][v] + _sensors[v].chrgCost()), _beta);
			prod_nodes_map.insert(make_pair(_taus[ant_k.cur_node][v] * eta_term, v));
		}
	}

	if (RU::uniformRand01() <= _q0) { sensor_idx = prod_nodes_map.begin()->second; }
	else {
		vector<double> prob_js; prob_js.resize(_num_nodes);
		for (auto it = prod_nodes_map.begin(); it != prod_nodes_map.end(); ++it) {
			prob_js[it->second] = it->first;
		}
		double neighb_sum = CommonUtils::vecElemsSum(prob_js);
		std::transform(prob_js.begin(), prob_js.end(), prob_js.begin(),
			[neighb_sum](double& c) -> double { return c / neighb_sum; });
		RU::vecBiasedSampleOneIndex(prob_js, prob_js, sensor_idx);
	}
	return sensor_idx;
}

void ACS::_pathConstruction(const size_t ant_idx)
{
	// Here we add sensors to the ant's path until all sensors are visited or
	// the path is infeasible.
	while ((!_ants[ant_idx].isAllVisited()) && _ants[ant_idx].isPathFeasible()) {
		size_t sensor_idx = _stateTransitionRule(_ants[ant_idx]);
		_localUpdatingRule(_ants[ant_idx].cur_node, sensor_idx);
		_ants[ant_idx].addSensorToPath(sensor_idx, _sensors[sensor_idx],
			_edge_costs[sensor_idx][END_IDX],
			_edge_costs[_ants[ant_idx].cur_node][sensor_idx]);
	}
	// Add last node to the ant's path.
	_ants[ant_idx].addLastToPath(_edge_costs[_ants[ant_idx].cur_node][END_IDX]);
}

void ACS::_2opt(const size_t ant_idx)
{
	auto delta_cost = [this](const vector<size_t>& path, 
		const size_t v1, const size_t v2) -> double
		{
			size_t v1_next = v1 + 1, v2_next = v2 + 1;
			double cost_2edge = (-_edge_costs[path[v1]][path[v1_next]]
				+ _edge_costs[path[v1]][path[v2]]
				- _edge_costs[path[v2]][path[v2_next]]
				+ _edge_costs[path[v1_next]][path[v2_next]]);

			double rev_cost = 0;
			if (v2 > v1_next) { //! Because the edge cost is asymmetric, add
				//! extra cost = (edge_cost[v1][v1_next] + ... + edge_cost[v2_prev][v2]
				//!          - edge_cost[v2][v2_prev] - ... - edge_cost[v1_next][v1])
				for (size_t i = v1_next; i < v2; i++) {
					rev_cost += (-_edge_costs[path[i]][path[i + 1]]
						+ _edge_costs[path[i + 1]][path[i]]);
				}
			}
			return cost_2edge + rev_cost;
		};

	bool improved = true;
	size_t cnt = 0;
	size_t path_len = _ants[ant_idx].path.size();
	while (improved && cnt < _num_iters_2opt) {
		improved = false;
		for (size_t i = 0; i < path_len - 2; ++i) {
			for (size_t j = i + 2; j < path_len - 1; ++j) { 
				//! j start from i + 2 because if j=i+1, then no swap 
				double cost_diff = delta_cost(_ants[ant_idx].path, i, j);
				if (cost_diff < 0) {
					CommonUtils::vecReverse(_ants[ant_idx].path, i, j);
					_ants[ant_idx].fit.cost += cost_diff;
					improved = true;
					cnt++;
				}
			}
		}
	}
}

void ACS::_dropNodesUntilPathFeasible(const size_t ant_idx)
{
	while (!_ants[ant_idx].isPathFeasible()) {
		// drop value, {node, cost difference}
		multimap<double, IdxVal> dropval_node_cost;
		// Then calculate the initial dropval_node_cost
		for (size_t j = 1; j < _ants[ant_idx].path.size() - 1; ++j) {
			size_t cur_node = _ants[ant_idx].path[j];
			size_t prev_node = _ants[ant_idx].path[j - 1];
			size_t next_node = _ants[ant_idx].path[j + 1];
			double sn_prize = _sensors[cur_node].prize();
			double sn_cost = _sensors[cur_node].chrgCost();
			double cost_diff = _edge_costs[cur_node][next_node]
				+ _edge_costs[prev_node][cur_node] + sn_cost
				- _edge_costs[prev_node][next_node];
			cost_diff = std::max(EPS, cost_diff);  // Avoid i is on line (i-1)(i+1)
			dropval_node_cost.insert(
				make_pair(sn_prize / cost_diff, make_pair(cur_node, cost_diff)));
		}
		size_t node2drop = dropval_node_cost.begin()->second.first;
		size_t idx2drop = CommonUtils::findIndexByElem(_ants[ant_idx].path, node2drop);
		// This is for the first drop value map update
		_ants[ant_idx].fit.prize -= _sensors[node2drop].prize();
		_ants[ant_idx].fit.cost -= dropval_node_cost.begin()->second.second;
		// Before erase the index, record its previous and next neighbor node.
		_ants[ant_idx].path.erase(_ants[ant_idx].path.begin() + idx2drop);
		// ALTHOUGH the dropped node performs POOR in current path, we still
		// need to add it back to unvisited node list because it may be a good 
		// candidate after adding some other nodes. 
		_ants[ant_idx].unmaskVisit(node2drop);
		_ants[ant_idx].updateFeasibility();
	}
}

double ACS::_addCost(const size_t adj0, const size_t adj1, 
	const size_t node2add) const
{
	return _edge_costs[adj0][node2add] + _edge_costs[node2add][adj1] 
		+ _sensors[node2add].chrgCost() - _edge_costs[adj0][adj1];
}

multimap<double, IdxIdxVal, std::greater<double>> ACS::_initAddValMap(
	const size_t ant_idx) const
{
	typedef tuple<size_t, size_t, size_t, size_t> QuadIdx;
	multimap<double, IdxIdxVal, std::greater<double>> addval_node_cost;
	for (size_t v = _num_depots; v < _num_nodes; ++v) {
		if (!_ants[ant_idx].isNodeVisited(v)) {
			// Distance, {node, node index in path}
			multimap<double, DualIdx> cost_nbrs;  
			// Find neighbor nodes of node, here we don't consider the 
			// charging cost because here is to find the closest neighbor.
			for (size_t j = 0; j < _ants[ant_idx].path.size(); j++) {
				cost_nbrs.insert(make_pair(_edge_costs[_ants[ant_idx].path[j]][v],
					make_pair(_ants[ant_idx].path[j], j)));
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
				if (nbr_idx < _ants[ant_idx].path.size() - 1) {
					size_t next_idx = nbr_idx + 1;
					cost_nbr_pairs.insert(make_pair(
						_addCost(nbr, _ants[ant_idx].path[next_idx], v), make_tuple(
							nbr, _ants[ant_idx].path[next_idx], nbr_idx, next_idx)));
				}
				if (nbr_idx > 0) {
					size_t prev_idx = nbr_idx - 1;
					cost_nbr_pairs.insert(make_pair(
						_addCost(_ants[ant_idx].path[prev_idx], nbr, v), make_tuple(
							_ants[ant_idx].path[prev_idx], nbr, prev_idx, nbr_idx)));
				}
			}
			nbr_pairs.clear();  // We don't need it any more.
			// Get the smallest addcost of this node v.
			double addcost_v = 1e6; size_t idx2add_v = 0;
			// If any two nodes adjacent in path, then find the pair w/
			// the SMALLEST addcost and find index to insert node v.
			addcost_v = cost_nbr_pairs.begin()->first;
			idx2add_v = std::max(  // left, node to add, right
				get<2>(cost_nbr_pairs.begin()->second),
				get<3>(cost_nbr_pairs.begin()->second));
			// If the addcost within budget constraint, add it into the addval_map
			if (addcost_v + _ants[ant_idx].fit.cost <= _uav0.engy + EPS) {
				addval_node_cost.insert(make_pair(_sensors[v].prize() / addcost_v,
					make_tuple(v, idx2add_v, addcost_v)));
			}
		}
	}
	return addval_node_cost;
}

void ACS::_iterativeAdd(const size_t ant_idx, size_t& idx2add, size_t& add_node,
	multimap<double, IdxIdxVal, std::greater<double>>& addval_node_cost)
{
	// add_value, {node2add, add_idx, add_cost}
	vector<std::pair<double, IdxIdxVal>> map2update;
	map2update.reserve(addval_node_cost.size());
	while (add_node != START_IDX) { // true means there exists feasible nodes to add
		// ONLY update prize and cost FOR NOW
		_ants[ant_idx].fit.prize += _sensors[add_node].prize();
		_ants[ant_idx].fit.cost += get<2>(addval_node_cost.begin()->second);
		addval_node_cost.erase(addval_node_cost.begin());  // Erase added node
		// Update map2update
		CommonUtils::eraseIf(addval_node_cost,
			[add_node, idx2add, this, ant_idx, &map2update](const auto& pair) -> bool
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
				double addcost_before_node2add = _addCost(
					_ants[ant_idx].path[idx2add - 1], add_node, unvis_node);
				temp_map.insert(make_pair(unvis_p / addcost_before_node2add,
					make_tuple(unvis_node, idx2add, addcost_before_node2add)));
				double addcost_aftr_node2add = _addCost(
					add_node, _ants[ant_idx].path[idx2add], unvis_node);
				temp_map.insert(make_pair(unvis_p / addcost_aftr_node2add,
					make_tuple(unvis_node, idx2add + 1, addcost_aftr_node2add)));
				// If meets the constraint, add it into addval_node_pair_cost and 
				// erase the iterator (because the value MAY not be updated)
				if (get<2>(temp_map.begin()->second) 
					+ _ants[ant_idx].fit.cost < _uav0.engy - EPS) {
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
		CommonUtils::vecInsertElem(_ants[ant_idx].path, idx2add, add_node);
		_ants[ant_idx].maskVisit(add_node);
		_ants[ant_idx].updateFeasibility();
		// Then get the CURRENT node2add
		if ((!addval_node_cost.empty()) && addval_node_cost.begin()->first > 0) {
			add_node = get<0>(addval_node_cost.begin()->second);
			idx2add = get<1>(addval_node_cost.begin()->second);
		}
		else { add_node = START_IDX; }
	}
}

void ACS::_addNodesUntilBudget(const size_t ant_idx)
{
	if ((!_ants[ant_idx].isAllVisited()) && _ants[ant_idx].isPathFeasible()) {
		size_t node2add = START_IDX, idx2add = START_IDX;
		// add_value, {node to add, path index to add, add_cost}
		multimap<double, IdxIdxVal, std::greater<double>> addval_node_cost = 
			_initAddValMap(ant_idx);
		if ((!addval_node_cost.empty()) && addval_node_cost.begin()->first > 0) {
			// Get the node v and index to add in current path (whose 
			// add value is positive and largest.
			node2add = get<0>(addval_node_cost.begin()->second);
			idx2add = get<1>(addval_node_cost.begin()->second);
		}
		else { node2add = START_IDX; }
		// Then iteratively add nodes until no room to add.
		_iterativeAdd(ant_idx, idx2add, node2add, addval_node_cost);
	}
}

void ACS::_localUpdatingRule(const size_t i, const size_t j)
{
	if (_taus[i][j] > _tau0) { _taus[i][j] -= _rho * (_taus[i][j] - _tau0); }
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

void ACS::_globalUpdatingRule()
{
	double evap_ratio = 1 - _alpha;
	for (size_t i = 0; i < _num_nodes; i++) {
		if (i != END_IDX) {  // Don't care tau[END_IDX][x]!
			for (size_t j = 1; j < _num_nodes; j++) {
				if (i != j) { _taus[i][j] *= evap_ratio; }
			}
		} 
	}
	for (size_t j = 0; j < _gb_ant->path.size() - 1; ++j) {
		_taus[_gb_ant->path[j]][_gb_ant->path[j + 1]] += _gb_ant->delta_tau;
	}
}

void ACS::evolveUntilStopCriteria()
{
	size_t num_targets = _num_nodes - _num_depots;
	vector<int> valid_nodes = vector<int>(num_targets);
	std::iota(valid_nodes.begin(), valid_nodes.end(), static_cast<int>(_num_depots));
	vector<int> start_nodes; start_nodes.reserve(_num_ants);

	for (size_t t = 0; t < _num_iters_acs; t++) {
		if (_no_impr_cnt > _max_no_impr) { break; }
		Fitness lb_fit = Fitness(1e8, -1);
		
		//! Uniformly sample start node for each ant.
		if (_num_ants > num_targets) {
			start_nodes.insert(
				start_nodes.end(), valid_nodes.begin(), valid_nodes.end());
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
				_edge_costs[idx1][END_IDX], _edge_costs[START_IDX][idx1]);
			//! [PATH CONSTRUCTION] Construct the solution path
			_pathConstruction(k);
			//! [2-OPT] Local search
			if (_num_iters_2opt > 0 && _ants[k].path.size() > 3) { 
				_2opt(k); 
				_ants[k].updateFeasibility(); 
			}
			//! [DROP NODE] Keep dropping node until path is feasible.
			_dropNodesUntilPathFeasible(k);
			//! [ADD NODE] Keep adding node until no unvisited nodes,
			// reach budget constraint, or reach number of searches.
			_addNodesUntilBudget(k);
			//! Determine the locally best ant for later updating globally best ant.
			if (_ants[k].fit > lb_fit) { lb_fit = _ants[k].fit; _lb_ant_id = k; }
		}
		_updateGlobalBestAnt();
		_globalUpdatingRule();
		if (t < _num_iters_acs - 1) {
			for (size_t k = 0; k < _num_ants; k++) { _ants[k].reset(_uav0); }
		}
	}
}

