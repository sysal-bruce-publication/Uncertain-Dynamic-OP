/*****************************************************************//**
 * \file   acs.h
 * \brief  Ant Colony System
 * 
 * \author Qiuchen Qian
 * \date   April 2024
 *********************************************************************/
#ifndef __ACS_H__
#define __ACS_H__

#include <iostream>
#include <map>
#include "ant.h"

typedef std::pair<size_t, double> IdxVal;
typedef std::pair<size_t, size_t> DualIdx;
typedef std::tuple<size_t, size_t, double> IdxIdxVal;

class ACS
{
public:
	ACS() = default;
	ACS(const size_t num_ants, const size_t max_no_impr, const size_t num_iters_acs,
		const size_t num_iters_2opt, const double q0, const double alpha,
		const double beta, const double rho, const double tol_acs, 
		const DroneState& uav0) :
		_num_ants(num_ants),
		_num_iters_acs(num_iters_acs),
		_num_iters_2opt(num_iters_2opt),
		_max_no_impr(max_no_impr),
		_q0(q0),
		_alpha(alpha),
		_beta(beta),
		_rho(rho),
		_tol_acs(tol_acs),
		_uav0(uav0) {}
	~ACS() {}

	double getBudget() const { return _uav0.engy; }
	Fitness getBestFitness() const { return _gb_ant->fit; }
	std::vector<size_t> getBestPath() const { return _gb_ant->path; }
	std::vector<std::vector<double>> getEdgeCosts() const { return _edge_costs; }
	void checkPathCostPrize(const std::vector<size_t>& path,
		const double cost_ref, const double prize_ref) const;
	double getPathCostWithWindModel(const std::vector<size_t>& path, 
		const Point2D& uni_wind) const;

	void initColony(std::vector<Sensor>& sensors);

	void initPhmoneMatrix();

	void evolveUntilStopCriteria();

private:
	size_t _lb_ant_id   = 0;
	size_t _num_nodes   = 0;
	size_t _no_impr_cnt = 0;
	size_t _num_ants    = 0;
	double _tau0        = 0;

	std::unique_ptr<GBAnt> _gb_ant;					// Global best ant
	std::vector<Sensor> _sensors;					// Sensors to visit
	std::vector<Ant> _ants;							// Ants in the colony	
	std::vector<std::vector<double>> _taus;			// Pheromone matrix
	std::vector<std::vector<double>> _edge_costs;	// Energy cost for visitation [kJ]
	DroneState _uav0;								// UAV with initial state.

	const size_t _num_iters_acs  = 0;	// Number of iterations of ACS
	const size_t _num_iters_2opt = 0;	// Number of iterations of 2-opt
	const size_t _max_no_impr    = 0;   // Maximum number of iterations without improvement
	const size_t _num_depots     = 2;	// Number of depot nodes, OP is 2
	const double _q0             = 0;	// Probability of choosing the best next node
	const double _alpha          = 0;	// Pheromone importance
	const double _beta           = 0;	// Heuristic importance
	const double _rho            = 0;	// Pheromone evaporation rate
	const double _tol_acs        = 0;	// Tolerance of improvement

	/**
	 * For the first planning, we assume uniform wind, therefore we can compute
	 * the cost matrix in advance.
	 */
	void _getInitEdgeCosts();

	/**
	 * Calculate the initial plan using nearest neighbor strategy.
	 * 
	 * \return 
	 */
	double _nearNgbrPrizeCostRatio() const;

	/**
	 * ACS State Transition Rule.
	 * If q <= q0, exploitation
	 * Else, biased exploration
	 *
	 * \param ant
	 * \return The picked node to visit
	 */
	size_t _stateTransitionRule(const Ant& ant_k) const;

	void _localUpdatingRule(const size_t i, const size_t j);

	void _pathConstruction(const size_t ant_idx);

	void _2opt(const size_t ant_idx);

	void _dropNodesUntilPathFeasible(const size_t ant_idx);

	/**
	 * Get the add cost based on node index.
	 *
	 * \param adj0
	 * \param adj1
	 * \param node2add
	 * \return
	 */
	double _addCost(const size_t adj0, const size_t adj1, 
		const size_t node2add) const;

	std::multimap<double, IdxIdxVal, std::greater<double>> _initAddValMap(
		const size_t ant_idx) const;

	void _iterativeAdd(const size_t ant_idx, size_t& idx2add, size_t& add_node,
		std::multimap<double, IdxIdxVal, std::greater<double>>& addval_map);

	void _addNodesUntilBudget(const size_t ant_idx);

	void _updateGlobalBestAnt();

	void _globalUpdatingRule();
};

#endif // !__ACS_H__

