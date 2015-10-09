#if !defined(TIMER_H)
#define TIMER_H

#include <chrono>

class Timer {
public:
	using Clock = std::chrono::high_resolution_clock;
	using Timepoint = std::chrono::time_point<std::chrono::high_resolution_clock>;
	using Duration = std::chrono::duration<long long, std::nano>;

	using Seconds = std::chrono::seconds;

	inline void start()
	{
		start_ = Clock::now();
	}

	inline Duration elapsed()
	{
		return (Clock::now() - start_);
	}

private:
	Timepoint start_;
};

#endif
