#if !defined(PLATFORM_H)
#define PLATFORM_H

#include <cstdlib>

#define UNUSED(a) ((void)(a))

#if defined(_WIN32)
    #define FINLINE __forceinline
#elif defined(__APPLE__)
    #define FINLINE inline __attribute__((always_inline))
#elif defined(__linux)
    #define FINLINE inline __attribute__((always_inline))
#else
    #error "Unsupported OS!"
#endif

template <typename T>
inline T* alignedAlloc(size_t numElements, int32_t alignment)
{
    const size_t allocSize = numElements * sizeof(T);

#if defined(_WIN32)
    return (T*)_aligned_malloc(allocSize, alignment);
#elif defined(__APPLE__) || defined(__linux)
    T* result;
    int err = posix_memalign((void**)&result, alignment, allocSize);
    return err == 0 ? (T*)result : nullptr;
#else
    #error "Unsupported OS!"
    return nullptr;
#endif
}

inline void alignedFree(void* ptr)
{
#if defined(_WIN32)
    _aligned_free(ptr);
#elif defined(__APPLE__) || defined(__linux)
    free(ptr);
#else
    #error "Unsupported OS!"
#endif
}

#endif // PLATFORM_H

