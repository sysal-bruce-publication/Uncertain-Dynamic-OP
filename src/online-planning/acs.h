#ifndef __ACS_H__
#define __ACS_H__

#include <map>
#include "ant.h"
#include "distb.h"

typedef std::pair<size_t, double> IdxVal;
typedef std::pair<size_t, size_t> DualIdx;
typedef std::tuple<size_t, size_t, double> IdxIdxVal;

class ACS
{
public:
	ACS() = default;
	/**
	 * Constructor for Ant colony System algorithm.
	 *
	 * \param num_ants
	 * \param max_no_impr
	 * \param num_iters_acs
	 * \param num_iters_2opt
	 * \param q0
	 * \param alpha
	 * \param beta
	 * \param rho
	 * \param tol_acs
	 * \param uav_E0
	 * \param uav_sample_dt
	 * \param uni_wind
	 * \param uav_coord
	 */
	ACS(const size_t start_idx, const size_t num_ants, 
		const size_t max_no_impr, const size_t num_iters_acs,
		const size_t num_iters_2opt, const double q0, const double alpha,
		const double beta, const double rho, const double tol_acs,
		const DroneState& uav0) :
		_start_idx(start_idx),
		_num_ants(num_ants),
		_num_iters_acs(num_iters_acs),
		_num_iters_2opt(num_iters_2opt),
		_max_no_impr(max_no_impr),
		_q0(q0),
		_alpha(alpha),
		_evap_rate(1 - alpha),
		_beta(beta),
		_rho(rho),
		_tol_acs(tol_acs),
		_uav0(uav0) {}
	~ACS() {}

	double getBudget() const { return _uav0.engy; }
	Fitness getBestFitness() const { return _gb_ant->fit; }
	bool isBestSolRisky() const {  return _gb_ant->path.size() < 4; }
	vector<size_t> getBestPath() const { return _gb_ant->path; }
	vector<vector<double>> getEdgeCosts() const { return _edge_engy_costs; }

	void checkSol(const Ant& ant_k) const;

	/**
	 * Initialize the online colony. This functions includes:
	 * 1. Update visitation mask from visited nodes in the previous simulation.
	 * 2. Update edge cost matrix from offline mission planning + online updating.
	 * 3. Initialize general ants and global-best ant. 
	 * 
	 * \param sensors
	 * \param visit_hist Visit history, including DEPOT_IDX.
	 * \param edge_costs
	 */
	void initColony(vector<Sensor>& sensors, vector<size_t>& visit_hist,
		vector<vector<double>>& edge_engy_costs);

	/**
	 * Update neighbor edges' costs based on weighted error.
	 * 
	 * \param prev_idx
	 * \param prev_engy
	 * \param weight
	 * \param bound
	 */
	void updateNeighbEdgeCostsWeightedAvg(const size_t prev_idx,
		const double prev_engy, const double act_weight, const double exp_weight);

	void updatePosteriorDistributions(const double alpha_0, const double kappa_0,
		const vector<double>& samples_tof, const vector<double>& samples_crs,
		const vector<double>& samples_ldg);
	void getUpdatedAvgPowers(const vector<double>& cdf_probs, 
		vector<AvgPower>& avg_powers) const;
	void updateNeighbEdgeCostsBayesian(const AvgPower& avg_pwr);
	void getUpdatedNeighbEdgeCostsBayesian(const AvgPower& avg_pwr, 
		vector<vector<double>>& new_costs) const;
	void updateNeighbEdgeCostsMCGreedy(const double p_t_len, const double p_c_len,
		const double p_l_len, const double p_t_min, const double p_c_min,
		const double p_l_min);

	/**
	 * Check whether base cost (start_idx -> end_idx) exceeds budget.
	 * 
	 * \return 
	 */
	bool isBudgetEnough(const double base_c) const;

	void setEdgeCosts(const vector<vector<double>>& edge_cost)
	{
		_edge_engy_costs = edge_cost;
	}

	/**
	 * Use global best path from last iteration to update 
	 * the initial pheromone matrix.
	 * 
	 * \param last_path
	 * \return 
	 */
	bool initPhmoneMatrix(vector<size_t>& last_path);

	void resetColony();

	void evolveUntilStopCriteria();

	double getPrizeSum() const;

private:
	size_t _num_nodes   = 0;
	size_t _lb_ant_id   = 0;
	size_t _no_impr_cnt = 0;
	size_t _num_ants    = 0;
	double _tau0        = 0;

	vector<Sensor> _sensors;			// Sensors to visit
	vector<Ant> _ants;					// Ants in the colony	
	vector<bool> _visit_mask0;			// Visit mask for iterative simulation
	vector<vector<double>> _taus;		// Pheromone matrix
	vector<vector<double>> _edge_engy_costs;  // Energy cost for visitation [kJ]
	std::unique_ptr<GBAnt> _gb_ant;		// Global best ant (with path and fitness)
	std::unique_ptr<LST> _lst_tof;		// Location-scale student's T for takeoff
	std::unique_ptr<LST> _lst_crs;		// Location-scale student's T for cruise
	std::unique_ptr<LST> _lst_ldg;		// Location-scale student's T for landing

	const size_t _num_iters_acs  = 0;	// Number of iterations of ACS
	const size_t _num_iters_2opt = 0;	// Number of iterations of 2-opt
	const size_t _max_no_impr    = 0;   // Maximum number of iterations without improvement
	const size_t _num_depots     = 2;	// Number of depot nodes, OP is 2
	const size_t _start_idx      = 0;	// Start node index
	const double _q0             = 0;	// Probability of choosing the best next node
	const double _alpha          = 0;	// Pheromone importance
	const double _evap_rate      = 0;	// Pheromone evaporation rate
	const double _beta           = 0;	// Heuristic importance
	const double _rho            = 0;	// Pheromone evaporation rate
	const double _tol_acs        = 0;	// Tolerance of improvement
	const DroneState _uav0;				// UAV with initial state.

	/**
	 * Get the drop cost based on node index to drop.
	 * drop cost := edge_cost[prev][curr] + edge_cost[curr][next] 
	 *	+ node_cost[curr] - edge_cost[prev][next]. 
	 * Ant's fitness cost will subtract this drop cost diff.
	 * 
	 * \param adj0
	 * \param adj1
	 * \param node2drop
	 * \return 
	 */
	double _getDropCost(const size_t adj0, const size_t adj1, 
		const size_t node2drop) const;

	void _dropNodesUntilPathFeasible(Ant& ant_k);

	/**
	 * Get the add cost based on node index. 
	 * add cost := edge_cost[prev][curr] + edge_cost[curr][next] 
	 *	+ node_cost[curr] - edge_cost[prev][next]. 
	 * Ant's fitness cost will add this add cost diff.
	 *
	 * \param adj0
	 * \param adj1
	 * \param node2add
	 * \return
	 */
	double _getAddCost(const size_t adj0, const size_t adj1,
		const size_t node2add) const;

	std::multimap<double, IdxIdxVal, std::greater<double>> _getInitAddValMap(
		const Ant& ant_k) const;

	void _iterativeAddNodes(Ant& ant_k, size_t& idx2add, size_t& add_node,
		std::multimap<double, IdxIdxVal, std::greater<double>>& addval_map);

	void _addNodesUntilBudget(Ant& ant_k);

	void _globalUpdatingRule();

	/**
	 * ACS State Transition Rule.
	 * If q <= q0, exploitation
	 * Else, biased exploration
	 *
	 * \param ant
	 * \return The picked node to visit
	 */
	size_t _stateTransitionRule(const Ant& ant_k) const;

	/**
	 * ACS local updating rule, this rule is to reduce the deposited pheromone on 
	 * visited edge. Eventually, it will reduce the edge to tau0 level so that each
	 * edge has similar chance to be visited. 
	 * 
	 * \param i
	 * \param j
	 */
	void _localUpdatingRule(const size_t i, const size_t j);

	void _pathConstruction(Ant& ant_k);

	void _2opt(Ant& ant_k);

	void _updateGlobalBestAnt();
};

#endif // !__ACS_H__

