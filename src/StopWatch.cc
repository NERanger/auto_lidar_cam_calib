#include "StopWatch.hpp"

using alcc::StopWatch;

StopWatch::StopWatch() {
	start_ = std::chrono::steady_clock::now();
}

double StopWatch::GetTimeElapse() {
	using std::chrono::duration;
	using std::chrono::steady_clock;
	using std::chrono::duration_cast;

	steady_clock::time_point stop = steady_clock::now();
	duration<double> elaspe = duration_cast<duration<double>>(stop - start_);

	return elaspe.count();
}