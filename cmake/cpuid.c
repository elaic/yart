#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define AVX_SHIFT 28
#define AVX_BIT (1 << AVX_SHIFT)

#define FMA_SHIFT 12
#define FMA_BIT (1 << FMA_SHIFT)

#define AVX2_SHIFT 5
#define AVX2_BIT (1 << AVX2_SHIFT)

static const int32_t CPUID_PROC_INFO = 0x1;
static const int32_t CPUID_EXTENDED_FEATURES = 0x7;

enum AvailableInstructions
{
    NO_AVX = 0,
    AVX = 1,
    AVX2 = 2,
    AVX_FMA = 3,
    AVX2_FMA = 4,
};

#if !defined(_MSC_VER)
void cpuid(int32_t regs[4], int32_t eax)
{
    __asm__("cpuid"
            :"=a"(regs[0]),
            "=b"(regs[1]),
            "=c"(regs[2]),
            "=d"(regs[3])
            :"a"(eax));
}
#define CPUIDFN cpuid
#else
#include <intrin.h>
#define CPUIDFN __cpuid
#endif

int main(void)
{
    int32_t featureBits[4] = { 0 } ;
    enum AvailableInstructions result = NO_AVX;

    CPUIDFN(featureBits, CPUID_PROC_INFO);

    if (featureBits[2] & AVX_BIT) {
        bool fmaSupported = featureBits[2] & FMA_BIT;

        CPUIDFN(featureBits, CPUID_EXTENDED_FEATURES);

        if (featureBits[1] & AVX2_BIT) {
            result = fmaSupported ? AVX2_FMA : AVX2;
        } else {
            result = fmaSupported ? AVX_FMA : AVX;
        }
    }

#if 0
    /* Useful for debug or adding new features */
    switch (result) {
        case NO_AVX:
            printf("No AVX support detected\n");
            break;
        case AVX:
            printf("AVX support detected\n");
            break;
        case AVX_FMA:
            printf("AVX with FMA support detected\n");
            break;
        case AVX2:
            printf("AVX2 support detected\n");
            break;
        case AVX2_FMA:
            printf("AVX2 with FMA support detected\n");
            break;
    }
#endif

    return result;
}

