#if !defined(TRIACCEL_H)
#define TRIACCEL_H

#include <cstdint>
#include <vector>

#include "vector.h"

#include "triangle.h"

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

static const int modulo[] = {1, 2, 0, 1};
__forceinline bool intersect(const TriAccel& triaccel, const Ray& ray,
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

#undef u
#undef v
}

#endif // TRIACCEL_H
