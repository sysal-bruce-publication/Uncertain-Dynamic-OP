/*****************************************************************//**
 * \file   utils.h
 * \brief  Utils functions
 *
 * \author Qiuchen Qian
 * \date   April 2024
 *********************************************************************/
#ifndef __UTILS_H__
#define __UTILS_H__

#include <set>
#include <cmath>
#include <cstddef>
#include <vector>
#include <iterator>
#include <random>
#include <algorithm> 
#include <numeric>
#include <memory>

constexpr double EPS = 1e-7;
constexpr double PI = 3.14159265;
using std::abs, std::size_t;

namespace CommonUtils
{
	/**
	 * Erase if condition meets.
	 *
	 * \param items
	 * \param predicate
	 */
	template<typename ContainerT, typename PredicateT>
	void eraseIf(ContainerT& items, const PredicateT& predicate) {
		for (auto it = items.begin(); it != items.end(); ) {
			if (predicate(*it)) it = items.erase(it);
			else ++it;
		}
	}

	/**
	 * Check whether all elements in the std::vector are unique.
	 *
	 * \param vec
	 * \return If unique, return true; otherwise, return false.
	 */
	template <typename T, typename A>
	inline bool isVecElemsUnique(const std::vector<T, A>& vec)
	{
		std::set<T> s(vec.begin(), vec.end());
		return s.size() == vec.size();
	}

	/**
	 * Check whether the bool std::vector is all true.
	 *
	 * \param vec
	 * \return
	 */
	inline bool isBoolVecAllTrue(const std::vector<bool>& vec)
	{
		return std::all_of(vec.begin(), vec.end(), [](bool b) { return b; });
	}

	/**
	 * Find the index of specific member in the std::vector.
	 *
	 * \param vec
	 * \param member
	 * \return
	 */
	template <typename T, typename A>
	inline int findIndexByElem(const std::vector<T, A>& vec, const T member)
	{
		auto it = std::find(vec.begin(), vec.end(), member);
		if (it != vec.cend()) {
			return static_cast<int>(std::distance(vec.begin(), it));
		}
		else { return -1; }
	}

	template <typename T, typename A>
	inline T vecMinElem(const std::vector<T, A>& vec)
	{
		T min_elem = *std::min_element(vec.begin(), vec.end());
		return min_elem;
	}

	template <typename T, typename A>
	inline T vecMaxElem(const std::vector<T, A>& vec)
	{
		T max_elem = *std::max_element(vec.begin(), vec.end());
		return max_elem;
	}

	template <typename T, typename A>
	inline T vecElemsSum(const std::vector<T, A>& vec)
	{
		T sum_res = 0;
		for (const T& elem : vec) { sum_res += elem; }
		return sum_res;
	}

	/**
	 * Erase one element from the std::vector.
	 *
	 * \param vec
	 * \param member
	 */
	template <typename T, typename A>
	inline void vecEraseElem(std::vector<T, A>& vec, const T member)
	{
		vec.erase(std::remove(vec.begin(), vec.end(), member), vec.end());
	}

	template <typename T, typename A>
	inline void vecInsertElem(std::vector<T, A>& vec, size_t idx_in_vec, const T item2insert)
	{
		vec.insert(vec.begin() + idx_in_vec, item2insert);
	}

	/**
	 * Take vec[v1+1] to vec[v2] and add them in reverse order.
	 *
	 * \param vec
	 * \param v1
	 * \param v2
	 */
	template <typename T, typename A>
	inline void vecReverse(std::vector<T, A>& vec,
		const std::size_t v1, const std::size_t v2)
	{
		std::reverse(vec.begin() + v1 + 1, vec.begin() + v2 + 1);
	}

	/**
	 * Re-range atan2 to range [0, 2pi].
	 *
	 * \return
	 */
	inline double rerangeAtan2(const double atan2_res)
	{
		return atan2_res >= 0 ? atan2_res : (2 * PI + atan2_res);
	}

	/**
	 * Restrict a value in range [val_min, val_max].
	 *
	 * \param val_min
	 * \param val_max
	 * \param value
	 */
	template<typename T>
	inline void restrictValueInRange(const T val_min, const T val_max, T& value)
	{
		value = value < val_min ? val_min : (value > val_max ? val_max : value);
	}
	/**
	 * Check if two variables have same sign. If one of them is 0, we
	 * assume they have same sign.
	 *
	 * \param a
	 * \param b
	 * \return
	 */
	template<typename T = double>
	inline bool isSameSign(const T a, const T b)
	{
		return (a * b < 0) ? false : true;
	}
}

namespace RandomUtils
{
	using std::random_device, std::mt19937;

	static double uniformRand01()
	{
		static std::random_device rd;
		static std::mt19937 engine(rd());
		std::uniform_real_distribution<double> distb(0., 1.);
		return distb(engine);
	}

	static int uniformRand(int val_min, int val_max) 
	{
		static std::random_device rd;
		static std::mt19937 engine(rd());
		std::uniform_int_distribution<int> distb(val_min, val_max);
		return distb(engine);
	}

	static double uniformRand(const double val_min, const double val_max) 
	{
		static std::random_device rd;
		static std::mt19937 engine(rd());
		std::uniform_real_distribution<double> distb(val_min, val_max);
		return distb(engine);
	}

	static size_t poissionRand(const double this_t, const double lambda)
	{
		static std::random_device rd;
		static std::mt19937 engine(rd());
		std::exponential_distribution<double> distb(1. / lambda);

		size_t count = 0; double t_sum = 0;
		while (t_sum <= this_t) {
			t_sum += distb(engine);
			if (t_sum <= this_t) count++;
		}
		return count;
	}

	static double normalRand(const double mean, const double std)
	{
		static std::random_device rd;
		static std::mt19937 engine(rd());
		std::normal_distribution<double> distb{ mean, std };
		return distb(engine);
	}

	template <typename T, typename A>
	static void vecUniformRandSampleOneElem(const std::vector<T, A>& vec_in, T& elem)
	{
		static std::random_device rd;
		static std::mt19937 engine(rd());
		std::uniform_int_distribution<size_t> distb(0, vec_in.size() - 1);
		elem = vec_in[distb(engine)];
	}

	template <typename T, typename A>
	static void vecUniformRandSample(const std::vector<T, A>& vec_in,
		const size_t num_elems, std::vector<T, A>& vec_out)
	{
		static std::random_device rd;
		static std::mt19937 engine(rd());
		std::sample(vec_in.begin(), vec_in.end(),
			std::back_inserter(vec_out), num_elems, engine);
	}

	/**
	 * Draw num_elems samples from a std::vector (without replacement)
	 * based on uniform distribution.
	 *
	 * \param vec_in
	 * \param num_elems
	 * \param vec_out
	 */
	template <typename T, typename A>
	static void vecUniformRandSampleExcludeFirstLast(const std::vector<T, A>& vec_in,
		const size_t num_elems, std::vector<T, A>& vec_out)
	{
		static std::random_device rd;
		static std::mt19937 engine(rd());
		std::sample(vec_in.begin() + 1, vec_in.end() - 1,
			std::back_inserter(vec_out), num_elems, engine);
	}

	/**
	 * Shuffle the std::vector, but skip first n elements.
	 *
	 * \param num_skip
	 * \param vec
	 */
	template<typename T, typename A>
	static void vecShuffleSkipFirstN(const size_t num_skip, std::vector<T, A>& vec)
	{
		static std::random_device rd;
		static std::mt19937 engine(rd());
		std::shuffle(vec.begin() + num_skip, vec.end(), engine);
	}

	template <typename T, typename A>
	static void vecBiasedSampleOneIndex(const std::vector<T, A>& vec_in,
		const std::vector<double>& weights, size_t& idx)
	{
		static std::random_device rd;
		static std::mt19937 engine(rd());
		std::discrete_distribution<size_t> distb{ weights.begin(), weights.end() };
		idx = distb(engine);
	}

	template <typename T, typename A>
	static void vecBiasedSampleOneElem(const std::vector<T, A>& vec_in,
		const std::vector<double>& weights, T& elem)
	{
		static std::random_device rd;
		static std::mt19937 engine(rd());
		std::discrete_distribution<size_t> distb{weights.begin(), weights.end()};
		elem = vec_in[distb(engine)];
	}
}

#endif // !__UTILS_H__