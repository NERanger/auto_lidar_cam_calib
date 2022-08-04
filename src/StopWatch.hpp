#pragma once

#include <chrono>

namespace alcc {
	class StopWatch {
	public:
		StopWatch();
		double GetTimeElapse();
	private:
		std::chrono::steady_clock::time_point start_;
	};
}