#if !defined(SCHEDULER_H)
#define SCHEDULER_H

#include <vector>

class Task {
public:
	virtual ~Task() { }

	virtual void run() = 0;
};

using WorkQueue = std::vector<std::unique_ptr<Task>>;

void enqueuTasks(WorkQueue& tasks);

void runTasks();

void waitForCompletion();

void workQueueInit();

void workQueueShutdown();

#endif // SCHEDULER_H

