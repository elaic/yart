#if !defined(TRIACCEL_H)
#define TRIACCEL_H

#include <cstdint>
#include <vector>

#include "platform.h"
#include "vector.h"
#include "vector8.h"

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
    Vector8 n_u;
    Vector8 n_v;
    Vector8 n_d;
	__m256i k;

    Vector8 b_u;
    Vector8 b_v;
    Vector8 b_d;
	__m256i triIdx;

    Vector8 c_u;
    Vector8 c_v;
    Vector8 c_d;
	__m256i meshIdx;

    BoolVector8 valid;
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

            accel8->n_u[j] = accel->n_u;
            accel8->n_v[j] = accel->n_v;
            accel8->n_d[j] = accel->n_d;
            accel8->k.m256i_i32[j] = accel->k;

            accel8->b_u[j] = accel->b_u;
            accel8->b_v[j] = accel->b_v;
            accel8->b_d[j] = accel->b_d;
            accel8->triIdx.m256i_i32[j] = accel->triIdx;

            accel8->c_u[j] = accel->c_u;
            accel8->c_v[j] = accel->c_v;
            accel8->c_d[j] = accel->c_d;
            accel8->meshIdx.m256i_i32[j] = accel->meshIdx;
            accel8->valid.set(j, true);
        }
    }

    TriAccel8* accel8 = &triaccel8[num8Chunks];
    for (int i = 0; i < 8; ++i) {
        const TriAccel* accel = &triaccel[num8Chunks * 8 + i];

        if (i < remainderTriangles) {
            accel8->n_u[i] = accel->n_u;
            accel8->n_v[i] = accel->n_v;
            accel8->n_d[i] = accel->n_d;
            accel8->k.m256i_i32[i] = accel->k;

            accel8->b_u[i] = accel->b_u;
            accel8->b_v[i] = accel->b_v;
            accel8->b_d[i] = accel->b_d;
            accel8->triIdx.m256i_i32[i] = accel->triIdx;

            accel8->c_u[i] = accel->c_u;
            accel8->c_v[i] = accel->c_v;
            accel8->c_d[i] = accel->c_d;
            accel8->meshIdx.m256i_i32[i] = accel->meshIdx;
            accel8->valid.set(i, true);
        } else {
            accel8->valid.set(i, false);
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
    Vector8 d_k;
    Vector8 d_ku;
    Vector8 d_kv;

    Vector8 o_k;
    Vector8 o_ku;
    Vector8 o_kv;

    // TODO: consider using _mm256_blend_ps for loading. Would require storing
    // ray data in __m256 structures
    for (int i = 0; i < 8; ++i) {
        if (triaccel.valid[i]) {
            auto k = triaccel.k.m256i_i32[i];
#define ku modulo[k]
#define kv modulo[k + 1]
            d_k[i] = ray.dir[k];
            d_ku[i] = ray.dir[ku];
            d_kv[i] = ray.dir[kv];

            o_k[i] = ray.orig[k];
            o_ku[i] = ray.orig[ku];
            o_kv[i] = ray.orig[kv];
#undef ku
#undef kv
        }
    }

    auto zero = Vector8(0.0f);
    auto one = Vector8(1.0f);
    auto minusone = Vector8(-1.0f);
    auto eps = Vector8(1e-4f);

    auto currT = Vector8(info->t);

    //const float nd = 1.0f / (ray.dir[triaccel.k]
    //	+ triaccel.n_u * ray.dir[ku] + triaccel.n_v * ray.dir[kv]);
    auto nd = one / fmadd(triaccel.n_v, d_kv, fmadd(triaccel.n_u, d_ku, d_k));

    //const float t = (triaccel.n_d - ray.orig[triaccel.k]
    //	- triaccel.n_u * ray.orig[ku] - triaccel.n_v * ray.orig[kv]) * nd;
    //auto t = (Vector8(triaccel.n_d) - o_k - fmsub(Vector8(triaccel.n_u), o_ku, Vector8(triaccel.n_v) * o_kv)) * nd;
    auto t = (triaccel.n_d - o_k - triaccel.n_u * o_ku - triaccel.n_v * o_kv) * nd;

    //if (!(info->t > t && t > 0.0001f)) return false;
    auto valid = t < currT && t > eps;
    valid = valid && triaccel.valid;
    if (none(valid)) return false;

    //const float hu = (ray.orig[ku] + t * ray.dir[ku]);
    //const float hv = (ray.orig[kv] + t * ray.dir[kv]);
    auto hu = fmadd(t, d_ku, o_ku);
    auto hv = fmadd(t, d_kv, o_kv);

    //const float lambda = (hu * triaccel.b_u + hv * triaccel.b_v + triaccel.b_d);
    auto lambda = fmadd(hu, triaccel.b_u, fmadd(hv, triaccel.b_v, triaccel.b_d));
    //if (lambda < 0.0f) return false;
    valid = valid && (lambda > zero);
    if (none(valid)) return false;

    //const float mue = (hu * triaccel.c_u + hv * triaccel.c_v + triaccel.c_d);
    auto mue = fmadd(hu, triaccel.c_u, fmadd(hv, triaccel.c_v, triaccel.c_d));
    //if (mue < 0.0f) return false;
    valid = valid && (mue > zero);
    if (none(valid)) return false;

    //if (lambda + mue > 1.0f) return false;
    valid = valid && ((lambda + mue) < one);
    if (none(valid)) return false;

    //info->t = t;
    //info->u = lambda;
    //info->v = mue;
    for (int i = 0; i < 8; ++i) {
        if (valid[i]) {
            if (t[i] < info->t) {
                info->t = t[i];
                info->u = lambda[i];
                info->v = mue[i];
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
