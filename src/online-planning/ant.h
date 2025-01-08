#ifndef __ANT_H__
#define __ANT_H__

#include "drone.h"

constexpr size_t DEPOT_IDX = 0;
constexpr size_t END_IDX   = 1;

struct Fitness
{
	Fitness() = default;
	Fitness(const double cost_, const double prize_) : cost(cost_), prize(prize_) {}

	Fitness& operator=(const Fitness& f) 
	{ 
		cost = f.cost; prize = f.prize; return *this;
	}
	bool operator<(const Fitness& other) const 
	{
		if (std::abs(prize - other.prize) > 1e-4) {
			return prize < other.prize;
		}
		return other.cost < cost;
	}
	// Other operators can be defined in terms of operator<
	bool operator>(const Fitness& other) const { return other < *this; }
	bool operator<=(const Fitness& other) const { return !(other < *this); }
	bool operator>=(const Fitness& other) const { return !(*this < other); }

	double cost = 0;
	double prize = 0;
};


struct GBAnt
{
	GBAnt() = default;

	double delta_tau = 0;
	Fitness fit = Fitness(1e8, -1);
	vector<size_t> path;

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
	Ant(const size_t start_idx, const size_t end_idx, const DroneState& uav0,
		const std::vector<bool>& visit_mask) :
		_start_idx(start_idx),
		cur_node(start_idx),
		_end_idx(end_idx),
		_bgt(uav0.engy),
		_uav(std::make_shared<Drone>(uav0))
	{  // Initialize the path and insert the start node.
		path.reserve(visit_mask.size()); 
		path.push_back(_start_idx);
		_visit_mask = visit_mask;
	}
	~Ant() {}

	size_t cur_node = DEPOT_IDX;
	Fitness fit = Fitness();
	vector<size_t> path;

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
	void reset(const DroneState& uav0, const std::vector<bool>& visit_mask);

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
		_feasible = fit.cost < _bgt - EPS ? true : false;
	}

private:
	bool _feasible              = true;	// Feasibility of the path
	std::shared_ptr<Drone> _uav = nullptr;	// UAV to simulate the energy consumption
	vector<bool> _visit_mask;

	const size_t _start_idx = 0;
	const size_t _end_idx   = 1;
	const double _bgt       = 0;   // Battery budget [kJ]
};

#endif // !__ANT_H__

