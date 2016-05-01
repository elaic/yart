#if !defined(VECTOR8_H)
#define VECTOR8_H

#include <cassert>

#include <immintrin.h>

#include "utils.h"

struct BoolVector8
{
    union {
        __m256 ymm;
        uint32_t scalar[8];
    };

    BoolVector8() = default;

    explicit BoolVector8(bool value)
        : ymm(_mm256_set1_ps(value ? convertBits<int, float>(0xffffffff) : 0.0f))
    { }

    explicit BoolVector8(const __m256& ymm)
        : ymm(ymm)
    { }

    FINLINE bool any() const
    {
        return _mm256_movemask_ps(ymm) != 0x00;
    }

    FINLINE bool all() const
    {
        return _mm256_movemask_ps(ymm) == 0xff;
    }

    FINLINE bool none() const
    {
        return _mm256_movemask_ps(ymm) == 0x00;
    }

    FINLINE BoolVector8 operator!() const
    {
        return BoolVector8(_mm256_xor_ps(_mm256_set1_ps(convertBits<int, float>(0xffffffff)), ymm));
    }

    FINLINE BoolVector8 operator&&(const BoolVector8& rhs) const
    {
        return BoolVector8(_mm256_and_ps(ymm, rhs.ymm));
    }

    FINLINE BoolVector8 operator||(const BoolVector8& rhs) const
    {
        return BoolVector8(_mm256_or_ps(ymm, rhs.ymm));
    }

    FINLINE bool operator[](int idx) const
    {
        assert(idx >= 0 && idx <= 7);
        return scalar[idx] != 0;
    }

    FINLINE void set(int idx, bool value)
    {
        assert(idx >= 0 && idx <= 7);
        scalar[idx] = value ? 0xffffffff : 0x0;
    }
};

static FINLINE bool any(const BoolVector8& bvec)
{
    return bvec.any();
}

static FINLINE bool all(const BoolVector8& bvec)
{
    return bvec.all();
}

static FINLINE bool none(const BoolVector8& bvec)
{
    return bvec.none();
}

struct IntVector8
{
    union {
        __m256i ymm;
        int32_t scalar[8];
    };

     IntVector8() = default;

     explicit IntVector8(int32_t val)
         : ymm(_mm256_set1_epi32(val))
     { }

     explicit IntVector8(const int32_t* vals)
         : ymm(_mm256_set_epi32(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6], vals[7]))
     { }

     explicit IntVector8(__m256i ymm)
         : ymm(ymm)
     { }

     IntVector8(const IntVector8& copy) = default;

     IntVector8& operator=(const IntVector8& copy) = default;

     IntVector8(IntVector8&& move) = default;

     IntVector8& operator=(IntVector8&& move) = default;

     FINLINE int32_t operator[](int idx) const
     {
         assert(idx >= 0 && idx <= 7);
         return scalar[idx];
     }

     FINLINE int32_t& operator[](int idx)
     {
         assert(idx >= 0 && idx <= 7);
         return scalar[idx];
     }
};

struct Vector8
{
    /*
     * TODO: perhaps better hide the __m256 in the internals of the class?
     * Is this union hack even a defined behaviour?
     */
    union {
        __m256 ymm;
        float scalar[8];
    };

    Vector8() = default;

    explicit Vector8(float val)
        : ymm(_mm256_set1_ps(val))
    { }

    explicit Vector8(const float* val)
        : ymm(_mm256_load_ps(val))
    { }

    explicit Vector8(const __m256& ymm)
        : ymm(ymm)
    { }

    Vector8(const Vector8& copy) = default;

    Vector8& operator=(const Vector8& copy) = default;

    Vector8(Vector8&& move) = default;

    Vector8& operator=(Vector8&& move) = default;

    FINLINE Vector8 operator-()
    {
        return Vector8(_mm256_sub_ps(_mm256_setzero_ps(), ymm));
    }

    FINLINE Vector8 operator+(const Vector8& rhs) const
    {
        return Vector8(_mm256_add_ps(ymm, rhs.ymm));
    }

    FINLINE Vector8 operator-(const Vector8& rhs) const
    {
        return Vector8(_mm256_sub_ps(ymm, rhs.ymm));
    }

    FINLINE Vector8 operator*(const Vector8& rhs) const
    {
        return Vector8(_mm256_mul_ps(ymm, rhs.ymm));
    }

    FINLINE Vector8 operator/(const Vector8& rhs) const
    {
        return Vector8(_mm256_div_ps(ymm, rhs.ymm));
    }

    FINLINE Vector8& operator+=(const Vector8& rhs)
    {
        ymm = _mm256_add_ps(ymm, rhs.ymm);
        return *this;
    }

    FINLINE Vector8& operator-=(const Vector8& rhs)
    {
        ymm = _mm256_sub_ps(ymm, rhs.ymm);
        return *this;
    }

    FINLINE Vector8& operator*=(const Vector8& rhs)
    {
        ymm = _mm256_mul_ps(ymm, rhs.ymm);
        return *this;
    }

    FINLINE Vector8& operator/=(const Vector8& rhs)
    {
        ymm = _mm256_div_ps(ymm, rhs.ymm);
        return *this;
    }

    FINLINE BoolVector8 operator==(const Vector8& rhs) const
    {
        return BoolVector8(_mm256_cmp_ps(ymm, rhs.ymm, _CMP_EQ_OQ));
    }

    FINLINE BoolVector8 operator!=(const Vector8& rhs) const
    {
        return BoolVector8(_mm256_cmp_ps(ymm, rhs.ymm, _CMP_NEQ_OQ));
    }

    FINLINE BoolVector8 operator>=(const Vector8& rhs) const
    {
        return BoolVector8(_mm256_cmp_ps(ymm, rhs.ymm, _CMP_GE_OQ));
    }

    FINLINE  BoolVector8 operator<=(const Vector8& rhs) const
    {
        return BoolVector8(_mm256_cmp_ps(ymm, rhs.ymm, _CMP_LE_OQ));
    }

    FINLINE BoolVector8 operator>(const Vector8& rhs) const
    {
        return BoolVector8(_mm256_cmp_ps(ymm, rhs.ymm, _CMP_GT_OQ));
    }

    FINLINE BoolVector8 operator<(const Vector8& rhs) const
    {
        return BoolVector8(_mm256_cmp_ps(ymm, rhs.ymm, _CMP_LT_OQ));
    }

    FINLINE float& operator[](int idx)
    {
        assert(idx >= 0 && idx <= 7);
        return scalar[idx];
    }

    FINLINE float operator[](int idx) const
    {
        assert(idx >= 0 && idx <= 7);
        return scalar[idx];
    }
};

static FINLINE Vector8 fmadd(const Vector8& mulLhs, const Vector8& mulRhs, const Vector8& add)
{
    return Vector8(_mm256_fmadd_ps(mulLhs.ymm, mulRhs.ymm, add.ymm));
}

static FINLINE Vector8 fmsub(const Vector8& mulLhs, const Vector8& mulRhs, const Vector8& sub)
{
    return Vector8(_mm256_fmsub_ps(mulLhs.ymm, mulRhs.ymm, sub.ymm));
}

#endif // VECTOR8_H
