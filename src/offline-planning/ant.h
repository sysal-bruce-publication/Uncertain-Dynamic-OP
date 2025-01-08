/*****************************************************************//**
 * \file   ant.h
 * \brief  General and global-best ant classes.
 * 
 * \author Qiuchen Qian
 * \date   April 2024
 *********************************************************************/
#ifndef __ANT_H__
#define __ANT_H__

#include "drone.h"

constexpr size_t START_IDX = 0;
constexpr size_t END_IDX   = 1;

struct Fitness
{
	Fitness() = default;
	Fitness(const double cost_, const double prize_): cost(cost_), prize(prize_) {}

	double cost = 0;
	double prize = 0;

	Fitness& operator=(const Fitness& f)
	{
		cost = f.cost; prize = f.prize; return *this;
	}
	bool operator>(const Fitness& f) const
	{
		if (prize > f.prize) { return true; }
		else if (abs(prize - f.prize) <= EPS && cost < f.cost) { return true; }
		else { return false; }
	}
};

struct GBAnt
{
	GBAnt() = default;

	double delta_tau = 0;
	Fitness fit = Fitness(1e8, -1);
	std::vector<size_t> path;

	void reset()
	{
		fit = Fitness(1e8, -1);
		if (!path.empty()) { path.clear(); }
	}
};


class Ant
{
public:
	Ant() = default;
	Ant(const size_t num_nodes, const DroneState& uav0) : 
		_bgt(uav0.engy),
		_uav(std::make_shared<Drone>(uav0))
	{
		// Initialize the path and insert the start node.
		path.reserve(num_nodes); path.push_back(START_IDX);
		// Initialize the visit mask and set the start and end nodes as visited.
		_visit_mask = std::vector<bool>(num_nodes, false);
		_visit_mask[START_IDX] = _visit_mask[END_IDX] = true;
	}
	~Ant() {}

	size_t cur_node = START_IDX;
	Fitness fit = Fitness();
	std::vector<size_t> path;

	/**
	 * Check whether the node at given index is visited.
	 * 
	 * \param idx
	 * \return 
	 */
	bool isNodeVisited(const size_t idx) const { return _visit_mask[idx]; }

	/**
	 * Check whether all nodes have been visited.
	 * 
	 * \return 
	 */
	bool isAllVisited() const { return CommonUtils::isBoolVecAllTrue(_visit_mask); }

	/**
	 * Check whether the complete path is feasible.
	 * 
	 * \param rth_cost
	 * \return 
	 */
	bool isPathFeasible() const { return _feasible; }

	/**
	 * Mask visit as true.
	 * 
	 * \param idx
	 */
	void maskVisit(const size_t idx) { _visit_mask[idx] = true; }

	/**
	 * Unmask visit as false.
	 * 
	 * \param idx
	 */
	void unmaskVisit(const size_t idx) { _visit_mask[idx] = false; }

	/**
	 * Reset this ant to initial state.
	 * 
	 * \param uav0
	 */
	void reset(const DroneState& uav0);

	/**
	 * Before adding the sensor into path, check whether this will violate budget.
	 * 
	 * \param sensor_idx
	 * \param sn
	 * \param rth_engy
	 * \param visit_engy
	 * 
	 * \return if safe, return true.
	 */
	bool safeAddSensorToPath(const size_t sensor_idx, const Sensor& sn,
		const double rth_engy, const double visit_engy);

	/**
	 * Add sensor to path during the PATH CONSTRUCTION phase.
	 * 
	 * \param sensor_idx
	 * \param sn
	 * \param rth_engy
	 * \param visit_engy
	 * \param visit_time
	 */
	void addSensorToPath(const size_t sensor_idx, const Sensor& sn,
		const double rth_engy, const double visit_engy);

	/**
	 * Include the Return-To-Home energy cost, and end index in the path.
	 * 
	 * \param rth_engy
	 */
	void addLastToPath(const double rth_engy);

	void updateFeasibility()
	{ 
		_feasible = fit.cost > _bgt + EPS ? false : true;
	}

private:
	bool _feasible = true;
	std::shared_ptr<Drone> _uav = nullptr;
	std::vector<bool> _visit_mask;

	const double _bgt = 0;   // Battery budget [kJ]
};

#endif // !__ANT_H__

