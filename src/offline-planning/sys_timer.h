/*****************************************************************//**
 * \file   Timer.h
 * \brief  System timer
 *
 * \author Bruce
 * \date   August 2023
 *********************************************************************/
#ifndef __SYS_TIMER_H__
#define __SYS_TIMER_H__

#include <chrono>
#include <assert.h>

using namespace std::chrono_literals;

class SysTimer
{
public:
	SysTimer() = default;
	~SysTimer() {}

	void tick()
	{
		_end = timep_t{};
		_start = std::chrono::steady_clock::now();
	}

	void tock() { _end = std::chrono::steady_clock::now(); }

	auto duration() const {
		// Use gsl_Expects if your project supports it.
		assert(_end != timep_t{} && "Timer must toc before reading the time");
		return std::chrono::duration_cast<std::chrono::milliseconds>(_end - _start);
	}

private:
	using timep_t = decltype(std::chrono::steady_clock::now());

	timep_t _start = std::chrono::steady_clock::now();
	timep_t _end = {};
};

#endif // !__SYS_TIMER_H__