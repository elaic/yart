#if !defined(SEMAPHORE_H)
#define SEMAPHORE_H

#include <condition_variable>
#include <cstdint>
#include <mutex>

class semaphore {
public:
	semaphore(int32_t count) : count_(count) { }

	~semaphore() = default;

	semaphore(const semaphore& copy) = delete;
	semaphore(semaphore&& move) = default;

	semaphore& operator=(const semaphore& copy) = delete;
	semaphore& operator=(semaphore&& move) = default;

	inline void post()
	{
		std::unique_lock<std::mutex> lock(mutex_);
		++count_;
		condition_.notify_one();
	}

	inline void wait()
	{
		std::unique_lock<std::mutex> lock(mutex_);
		while (count_ == 0)
			condition_.wait(lock);
		--count_;
	}

private:
	int32_t count_;
	std::mutex mutex_;
	std::condition_variable condition_;
};

#endif // SEMAPHORE_H