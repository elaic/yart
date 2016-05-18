#if !defined(BBOX_H)
#define BBOX_H

#include <algorithm>

#include "vector.h"

struct BBox {
    Vector3f min;
    Vector3f max;

    BBox() = default;

    BBox(const BBox& copy) = default;

    BBox(BBox&& move) = default;

    explicit BBox(Vector3f& point) : min(point), max(point) { };

    BBox& operator=(const BBox& copy) = default;

    BBox& operator=(BBox&& move) = default;

    BBox(const Vector3f& min, const Vector3f& max) : min(min), max(max) { }

    bool intersect(const Ray& ray, float* tnear = nullptr, float* tfar = nullptr) const
    {
        float tNear = ray.minT;
        float tFar = ray.maxT;

        for (int i = 0; i < 3; ++i) {
            float invDir = 1.0f / ray.dir[i];
            float t0 = (min[i] - ray.orig[i]) * invDir;
            float t1 = (max[i] - ray.orig[i]) * invDir;

            if (t1 < t0) std::swap(t1, t0);

            if (tNear < t0) tNear = t0;
            if (tFar > t1) tFar = t1;
            if (tFar < tNear) return false;
        }

        if (tnear) *tnear = tNear;
        if (tfar) *tfar = tFar;
        return true;
    }
};

inline BBox boxUnion(const BBox& box, const Vector3f point)
{
    using std::min;
    using std::max;

    return BBox(
            Vector3f(min(box.min.x, point.x), min(box.min.y, point.y), min(box.min.z, point.z)),
            Vector3f(max(box.max.x, point.x), max(box.max.y, point.y), max(box.max.z, point.z)));
}

inline BBox boxUnion(const BBox& lhs, const BBox& rhs)
{
    using std::min;
    using std::max;

    BBox ret = {
        Vector3f(min(lhs.min.x, rhs.min.x), min(lhs.min.y, rhs.min.y), min(lhs.min.z, rhs.min.z)),
        Vector3f(max(lhs.max.x, rhs.max.x), max(lhs.max.y, rhs.max.y), max(lhs.max.z, rhs.max.z))
    };
    return ret;
}

#endif // !defined(BBOX_H)
