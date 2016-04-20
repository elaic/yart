#if !defined(TRIACCEL_H)
#define TRIACCEL_H

#include <cstdint>
#include <vector>

#include "platform.h"
#include "vector.h"

#include "triangle.h"

// AVX2
#include <immintrin.h>

struct TriAccel {
	// plane
	float n_u;
	float n_v;
	float n_d;
	int32_t k; // projection dimension

	float b_u;
	float b_v;
	float b_d;
	int32_t triIdx;

	float c_u;
	float c_v;
	float c_d;
	int32_t meshIdx;
};

// AVX based Wald triangle ray intersection
struct TriAccel8 {
	__m256 n_u;
	__m256 n_v;
	__m256 n_d;
	__m256i k;

	__m256 b_u;
	__m256 b_v;
	__m256 b_d;
	__m256i triIdx;

	__m256 c_u;
	__m256 c_v;
	__m256 c_d;
	__m256i meshIdx;

    __m256 valid;
};

inline void project(TriAccel* const triaccel, const Triangle& triangle,
	const std::vector<Vector3f>& vertices, int32_t triangleIdx, int32_t meshIdx)
{
	using std::abs;

	// Calculate geometric normal
	const auto& a = vertices[triangle.idx0];
	const auto& b = vertices[triangle.idx1];
	const auto& c = vertices[triangle.idx2];

	const auto& ab = b - a;
	const auto& ac = c - a;
	const auto& normal = cross(ab, ac);

	// Choose dimension in which normal has the largest absolute value
	// by default project on XZ axis
	auto k = 1;
	if (abs(normal.x) > abs(normal.y)) {
		if (abs(normal.x) > abs(normal.z)) {
			// project on YZ axis
			k = 0;
		} else {
			// project on XY axis
			k = 2;
		}
	} else {
		if (abs(normal.z) > abs(normal.y)) {
			// project on XY axis
			k = 2;
		}
	}

	auto u = (k + 1) % 3;
	auto v = (k + 2) % 3;

	const auto& normProj = normal / normal[k];

	// fill in the accel triangle data
	triaccel->n_u = normProj[u];
	triaccel->n_v = normProj[v];
	triaccel->n_d = dot(a, normProj);
	triaccel->k   = k;

	const auto det = ab[u] * ac[v] - ab[v] * ac[u];

	triaccel->b_u =  -ab[v] / det;
	triaccel->b_v =   ab[u] / det;
	triaccel->b_d =  (ab[v] * a[u] - ab[u] * a[v]) / det;
	triaccel->triIdx = triangleIdx;

	triaccel->c_u =   ac[v] / det;
	triaccel->c_v =  -ac[u] / det;
	triaccel->c_d = -(ac[v] * a[u] - ac[u] * a[v]) / det;
	triaccel->meshIdx = meshIdx;
}

// Just for debug use TriAccel to load TriAccel8
inline void loadTriaccel8(
    TriAccel8* const      triaccel8,
    const TriAccel* const triaccel,
    size_t                numTriangles)
{
    auto num8Chunks = numTriangles / 8;
    auto remainderTriangles = numTriangles % 8;

    for (size_t i = 0; i < num8Chunks; ++i) {
        TriAccel8* accel8 = &triaccel8[i];

        for (int j = 0; j < 8; ++j) {
            const TriAccel* accel = &triaccel[i * 8 + j];

            accel8->n_u.m256_f32[j] = accel->n_u;
            accel8->n_v.m256_f32[j] = accel->n_v;
            accel8->n_d.m256_f32[j] = accel->n_d;
            accel8->k.m256i_i32[j] = accel->k;

            accel8->b_u.m256_f32[j] = accel->b_u;
            accel8->b_v.m256_f32[j] = accel->b_v;
            accel8->b_d.m256_f32[j] = accel->b_d;
            accel8->triIdx.m256i_i32[j] = accel->triIdx;

            accel8->c_u.m256_f32[j] = accel->c_u;
            accel8->c_v.m256_f32[j] = accel->c_v;
            accel8->c_d.m256_f32[j] = accel->c_d;
            accel8->meshIdx.m256i_i32[j] = accel->meshIdx;
            accel8->valid.m256_f32[j] = -1.0f;
        }
    }

    TriAccel8* accel8 = &triaccel8[num8Chunks];
    for (size_t i = 0; i < 8; ++i) {
        const TriAccel* accel = &triaccel[num8Chunks * 8 + i];

        if (i < remainderTriangles) {
            accel8->n_u.m256_f32[i] = accel->n_u;
            accel8->n_v.m256_f32[i] = accel->n_v;
            accel8->n_d.m256_f32[i] = accel->n_d;
            accel8->k.m256i_i32[i] = accel->k;

            accel8->b_u.m256_f32[i] = accel->b_u;
            accel8->b_v.m256_f32[i] = accel->b_v;
            accel8->b_d.m256_f32[i] = accel->b_d;
            accel8->triIdx.m256i_i32[i] = accel->triIdx;

            accel8->c_u.m256_f32[i] = accel->c_u;
            accel8->c_v.m256_f32[i] = accel->c_v;
            accel8->c_d.m256_f32[i] = accel->c_d;
            accel8->meshIdx.m256i_i32[i] = accel->meshIdx;
            accel8->valid.m256_f32[i] = -1.0f;
        } else {
            accel8->valid.m256_f32[i] = 1.0f;
        }
    }
}

static const int modulo[] = {1, 2, 0, 1};
FINLINE bool intersect(const TriAccel& triaccel, const Ray& ray,
	RayHitInfo* const info)
{
#define ku modulo[triaccel.k]
#define kv modulo[triaccel.k + 1]

	const float nd = 1.0f / (ray.dir[triaccel.k]
		+ triaccel.n_u * ray.dir[ku] + triaccel.n_v * ray.dir[kv]);
	const float t = (triaccel.n_d - ray.orig[triaccel.k]
		- triaccel.n_u * ray.orig[ku] - triaccel.n_v * ray.orig[kv]) * nd;

	if (!(info->t > t && t > 0.0001f)) return false;

	const float hu = (ray.orig[ku] + t * ray.dir[ku]);
	const float hv = (ray.orig[kv] + t * ray.dir[kv]);

	const float lambda = (hu * triaccel.b_u + hv * triaccel.b_v + triaccel.b_d);
	if (lambda < 0.0f) return false;

	const float mue = (hu * triaccel.c_u + hv * triaccel.c_v + triaccel.c_d);
	if (mue < 0.0f) return false;

	if (lambda + mue > 1.0f) return false;

	info->t = t;
	info->u = lambda;
	info->v = mue;
	return true;

#undef ku
#undef kv
}

#pragma warning (push)
// Supress potentially uninitialized local variable for ray data. They are only
// uninitialized when data in triaccel8 is not valid
#pragma warning (disable: 4701)
// Needs Ray8 adapter, to avoid loading the same values over and over again
// https://software.intel.com/sites/landingpage/IntrinsicsGuide
FINLINE bool intersect(const TriAccel8& triaccel, const Ray& ray,
    RayHitInfo* const info, int* chunk8Idx)
{
    __m256 d_k;
    __m256 d_ku;
    __m256 d_kv;

    __m256 o_k;
    __m256 o_ku;
    __m256 o_kv;

    // TODO: consider using _mm256_blend_ps for loading. Would require storing
    // ray data in __m256 structures
    for (int i = 0; i < 8; ++i) {
        if (triaccel.valid.m256_f32[i] == -1.0f)
        {
            auto k = triaccel.k.m256i_i32[i];
#define ku modulo[k]
#define kv modulo[k + 1]
            d_k.m256_f32[i] = ray.dir[k];
            d_ku.m256_f32[i] = ray.dir[ku];
            d_kv.m256_f32[i] = ray.dir[kv];

            o_k.m256_f32[i] = ray.orig[k];
            o_ku.m256_f32[i] = ray.orig[ku];
            o_kv.m256_f32[i] = ray.orig[kv];
#undef ku
#undef kv
        }
    }

    __m256 zero = _mm256_set1_ps(0.0f);
    __m256 one = _mm256_set1_ps(1.0f);
    __m256 minusone = _mm256_set1_ps(-1.0f);
    __m256 eps = _mm256_set1_ps(1e-4f);

    //const float nd = 1.0f / (ray.dir[triaccel.k]
    //	+ triaccel.n_u * ray.dir[ku] + triaccel.n_v * ray.dir[kv]);
    __m256 tmp1 = _mm256_fmadd_ps(triaccel.n_u, d_ku, d_k);
    __m256 tmp2 = _mm256_fmadd_ps(triaccel.n_v, d_kv, tmp1);
    __m256 nd = _mm256_div_ps(one, tmp2);

    //const float t = (triaccel.n_d - ray.orig[triaccel.k]
    //	- triaccel.n_u * ray.orig[ku] - triaccel.n_v * ray.orig[kv]) * nd;
    __m256 tmpt1 = _mm256_mul_ps(triaccel.n_v, o_kv);
    __m256 tmpt2 = _mm256_mul_ps(triaccel.n_u, o_ku);
    __m256 tmpt3 = _mm256_sub_ps(triaccel.n_d, o_k);
    __m256 tmpt4 = _mm256_sub_ps(tmpt3, tmpt2);
    __m256 tmpt5 = _mm256_sub_ps(tmpt4, tmpt1);
    __m256 t = _mm256_mul_ps(tmpt5, nd);

    //if (!(info->t > t && t > 0.0001f)) return false;
    __m256 currT = _mm256_broadcast_ss(&info->t);
    __m256 cmp1 = _mm256_cmp_ps(t, currT, _CMP_LT_OQ);
    __m256 cmp2 = _mm256_cmp_ps(t, eps, _CMP_GT_OQ);
    __m256 valid1 = _mm256_and_ps(cmp1, cmp2);
    __m256 valid666 = _mm256_and_ps(valid1, triaccel.valid);
    __m256 cmpVal = _mm256_and_ps(valid666, minusone);
    if (_mm256_movemask_ps(cmpVal) == 0) return false;

    //const float hu = (ray.orig[ku] + t * ray.dir[ku]);
    //const float hv = (ray.orig[kv] + t * ray.dir[kv]);
    __m256 hu = _mm256_fmadd_ps(t, d_ku, o_ku);
    __m256 hv = _mm256_fmadd_ps(t, d_kv, o_kv);

    //const float lambda = (hu * triaccel.b_u + hv * triaccel.b_v + triaccel.b_d);
    __m256 tmp7 = _mm256_fmadd_ps(triaccel.b_v, hv, triaccel.b_d);
    __m256 lambda = _mm256_fmadd_ps(triaccel.b_u, hu, tmp7);
    //if (lambda < 0.0f) return false;
    __m256 cmp3 = _mm256_cmp_ps(lambda, zero, _CMP_GT_OQ);
    __m256 valid2 = _mm256_and_ps(valid1, cmp3);
    cmpVal = _mm256_and_ps(valid2, minusone);
    if (_mm256_movemask_ps(cmpVal) == 0) return false;

    //const float mue = (hu * triaccel.c_u + hv * triaccel.c_v + triaccel.c_d);
    __m256 tmp8 = _mm256_fmadd_ps(triaccel.c_v, hv, triaccel.c_d);
    __m256 mue = _mm256_fmadd_ps(triaccel.c_u, hu, tmp8);
    //if (mue < 0.0f) return false;
    __m256 cmp4 = _mm256_cmp_ps(mue, zero, _CMP_GT_OQ);
    __m256 valid3 = _mm256_and_ps(valid2, cmp4);
    cmpVal = _mm256_and_ps(valid3, minusone);
    if (_mm256_movemask_ps(cmpVal) == 0) return false;

    //if (lambda + mue > 1.0f) return false;
    __m256 tmp9 = _mm256_add_ps(lambda, mue);
    __m256 cmp5 = _mm256_cmp_ps(tmp9, one, _CMP_LT_OQ);
    __m256 valid4 = _mm256_and_ps(valid3, cmp5);
    cmpVal = _mm256_and_ps(valid4, minusone);
    int validMask = _mm256_movemask_ps(cmpVal);
    if (validMask == 0) return false;

    //info->t = t;
    //info->u = lambda;
    //info->v = mue;
    for (int i = 0; i < 8; ++i) {
        if (validMask & (1 << i)) {
            if (t.m256_f32[i] < info->t) {
                info->t = t.m256_f32[i];
                info->u = lambda.m256_f32[i];
                info->v = mue.m256_f32[i];
                assert(info->u >= 0.0f);
                assert(info->v >= 0.0f);
                assert(info->u + info->v <= 1.0f);
                *chunk8Idx = i;
            }
        }
    }
    return true;
}
#pragma warning (pop)

#endif // TRIACCEL_H
