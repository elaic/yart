#if !defined(UTILS_H)
#define UTILS_H

#include "platform.h"

template <unsigned int n>
inline float pow(float val)
{
	return val * pow<n - 1>(val);
}

template<>
inline float pow<1>(float val)
{
	return val;
}

template<>
inline float pow<0>(float val)
{
	UNUSED(val);
	return 1.0f;
}

#endif // UTILS_H