#if !defined(PLATFORM_H)
#define PLATFORM_H

#if defined(_WIN32)
    #define FINLINE __forceinline
    #define ALLOC_ALIGNED(ptr, alignment, size) \
        *(ptr) = _aligned_malloc((size), (alignment));
    #define FREE_ALIGNED(ptr) _aligned_free(ptr)
#elif defined(__APPLE__)
    #define FINLINE inline __attribute__((always_inline))
    #include <cstdlib>
    #define ALLOC_ALIGNED(ptr, alignment, size) posix_memalign((ptr), (alignment), (size))
    #define FREE_ALIGNED(ptr) free(ptr)
#elif defined(__linux)
    #define FINLINE inline __attribute__((always_inline))
    #define ALLOC_ALIGNED(ptr, alignment, size) posix_memalign((ptr), (alignment), (size))
    #define FREE_ALIGNED(ptr) free(ptr)
#else
    #error "Unsupported OS!"
#endif

#define UNUSED(a) ((void)(a))

#endif // PLATFORM_H

