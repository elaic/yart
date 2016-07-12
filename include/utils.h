#if !defined(UTILS_H)
#define UTILS_H

#include "platform.h"

#include <cstring>

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

template <typename T>
inline float lerp(T min, T max, float t)
{
    return (1.0f - t) * min + t * max;
}

template <typename TSrc, typename TDst>
inline TDst convertBits(TSrc src)
{
    TDst result;
    std::memcpy(&result, &src, sizeof(src));
    return result;
}

#endif // UTILS_H
