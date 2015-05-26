#if !defined(RNG_H)
#define RNG_H

#include <random>

class Rng {
public:
	Rng(int32_t seed = 1234) : rng_(seed)
	{ }

	int32_t randomInt()
	{
		return distInt_(rng_);
	}

	uint32_t randomUInt()
	{
		return distUint_(rng_);
	}

	float randomFloat()
	{
		return distFloat_(rng_);
	}

private:
	std::mt19937_64 rng_;
	std::uniform_int_distribution<int32_t> distInt_;
	std::uniform_int_distribution<uint32_t> distUint_;
	std::uniform_real_distribution<float> distFloat_;
};

#endif // RNG_H
