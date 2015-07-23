#if !defined(SCHEDULER_H)
#define SCHEDULER_H

#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

class Task
{
public:
	virtual void run() = 0;
};

class Scheduler {
public:
	Scheduler();

private:
	void scheduleWork();

private:
	std::deque<Task> taskQueue_;
	std::vector<std::thread> threads_;
	std::mutex taskMutex_;
	std::condition_variable taskCondition;
};

#endif // SCHEDULER_H

