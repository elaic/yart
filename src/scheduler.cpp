#include "scheduler.h"

#include <algorithm>
#include <cstdint>
#include <thread>

#include "semaphore.h"

std::vector<std::thread> workers;

WorkQueue workQueue;
using LockGuard = std::unique_lock<std::mutex>;
std::mutex queueMutex;
std::mutex runMutex;
std::condition_variable runCondition;
semaphore taskSemahore(0);

size_t numUnfinished = 0;

static const int32_t numWorkers = 8;

void enqueuTasks(WorkQueue& tasks)
{
    {
        LockGuard lock(queueMutex);
        for (auto& task : tasks) {
            workQueue.push_back(std::move(task));
        }
        printf("Done enqueueing tasks\n");
    }
    {
        LockGuard lock(runMutex);
        numUnfinished = workQueue.size();
    }
}

void runTasks()
{
    printf("Running tasks\n");
    auto size = workQueue.size();
    while (size-- > 0) {
        taskSemahore.post();
    }
}

void taskEntry()
{
    for (;;) {
        taskSemahore.wait();

        std::unique_ptr<Task> currentTile;
        {
            LockGuard lock(queueMutex);
            if (workQueue.size() == 0)
                break;
            currentTile = std::move(workQueue.back());
            workQueue.pop_back();
        }

        currentTile->run();

        {
            LockGuard lock(runMutex);
            auto unfinished = --numUnfinished;
            if (unfinished <= 0)
            {
                runCondition.notify_one();
                printf("Tasks finished\n");
            }
        }
    }
}

void waitForCompletion()
{
    printf("Wait for task completion\n");
    LockGuard lock(runMutex);
    while (numUnfinished > 0)
        runCondition.wait(lock);
}

void workQueueInit()
{
    printf("Init work queue\n");
    workers.reserve(numWorkers);
    for (int32_t i = 0; i < numWorkers; ++i) {
        workers.push_back(std::thread(taskEntry));
    }
}

void workQueueShutdown()
{
    printf("Shutdown work queue\n");
    waitForCompletion();
    for (int32_t i = 0; i < numWorkers; ++i) {
        taskSemahore.post();
    }
    std::for_each(begin(workers), end(workers), [&](auto& t){ t.join(); });
}
