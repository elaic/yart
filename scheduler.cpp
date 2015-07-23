#include "scheduler.h"

#include <cstdio>

static void dispatchThread(class Thread*);

class Thread {
public:
	Thread()
	{
		thread_ = std::thread(dispatchThread, this);
	}

	virtual void run() = 0;

private:
	std::thread thread_;
};

void dispatchThread(Thread* thr)
{
	thr->run();
}

class WorkerThread : public Thread {
	void run() override
	{
		printf("Hello, world!");
	}
};

void Scheduler::scheduleWork()
{

}

