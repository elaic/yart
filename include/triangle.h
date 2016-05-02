#if !defined(TRIANGLE_H)
#define TRIANGLE_H

#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "platform.h"

#include "bsdf.h"
#include "utils.h"
#include "vector.h"

// This is the traditional triangle implementation. While it is memory
// efficient, it would seem that it is quite cache unfriendly. Perhaps
// implementation where all vertices are stored by value should be tested...
struct Triangle {
    int32_t idx0;
    int32_t idx1;
    int32_t idx2;

    Triangle(int32_t id0, int32_t id1, int32_t id2)
        : idx0(id0), idx1(id1), idx2(id2)
    { }

    Triangle(const Triangle& copy) = default;
    Triangle(Triangle&& move) = default;

    Triangle& operator=(const Triangle& copy) = default;
    Triangle& operator=(Triangle&& move) = default;
};

// Moller-Trumbore ray triangle intersection
// Winding order is counter clock wise (ccw)
// TODO: test for performance against raw array of points
FINLINE bool intersect(const Ray& ray, const Triangle& triangle,
    const std::vector<Vector3f>& points, RayHitInfo* const hitInfo)
{
    const auto& v0 = points[triangle.idx0];
    const auto& v1 = points[triangle.idx1];
    const auto& v2 = points[triangle.idx2];

    const auto& e1 = v1 - v0;
    const auto& e2 = v2 - v0;

    const auto& pvec = cross(ray.dir, e2);
    const auto det = dot(e1, pvec);

    if (det > -EPS && det < EPS)
        return false;

    const auto invDet = 1.0f / det;

    const auto& tvec = ray.orig - v0;
    hitInfo->u = dot(tvec, pvec) * invDet;
    if (hitInfo->u < 0 || hitInfo->u > 1)
        return false;

    const auto& qvec = cross(tvec, e1);
    hitInfo->v = dot(ray.dir, qvec) * invDet;
    if (hitInfo->v < 0 || (hitInfo->u + hitInfo->v) > 1)
        return false;

    hitInfo->t = dot(e2, qvec) * invDet;

    return true;
}

class TriangleMesh {
public:
    using tri_size_t = std::vector<Triangle>::size_type;

public:
    TriangleMesh(const std::vector<Vector3f>& vertices,
        const std::vector<Triangle>& triangles,
        const std::shared_ptr<Bsdf> bsdf)
        : vertices_(vertices)
        , triangles_(triangles)
        , bsdf_(bsdf)
    {
        std::unordered_map<int32_t, std::unordered_set<int32_t>> vertexTriangleMap;

        for (size_t i = 0; i < triangles.size(); ++i) {
            vertexTriangleMap[triangles_[i].idx0].emplace((int32_t)i);
            vertexTriangleMap[triangles_[i].idx1].emplace((int32_t)i);
            vertexTriangleMap[triangles_[i].idx2].emplace((int32_t)i);
        }

        assert(vertices_.size() >= vertexTriangleMap.size());
        normals_.reserve(vertices_.size());

        for (size_t i = 0; i < vertexTriangleMap.size(); ++i) {
            auto normal = Vector3f(0);

            for (const auto& triIdx : vertexTriangleMap[(int32_t)i]) {
                normal += getNormal(triIdx);
            }

            normals_.push_back(normal / (float)vertexTriangleMap[(int32_t)i].size());
        }
    }

    inline bool intersect(const Ray& ray, RayHitInfo* const hitInfo) const
    {
        RayHitInfo localHitInfo;
        auto currentT = std::numeric_limits<float>::max();
        auto hitId = -1;
        for (tri_size_t i = 0; i < triangles_.size(); ++i) {
            if (::intersect(ray, triangles_[i], vertices_, &localHitInfo) &&
                localHitInfo.t < currentT && localHitInfo.t > 0.0f) {
                *hitInfo = localHitInfo;
                currentT = localHitInfo.t;
                hitId = (int)i;
            }
        }

        // Calculate geometric normal if a triangle was hit
        if (hitId >= 0) {
            hitInfo->normal = getNormal(hitId);
            hitInfo->bsdf = bsdf_.get();
        }
        return currentT < std::numeric_limits<float>::max() && currentT > 0.0f;
    }

	inline Vector3f getNormal(int32_t triangleIdx) const
	{
		const auto& t = triangles_[triangleIdx];
		const auto& e1 = vertices_[t.idx1] - vertices_[t.idx0];
		const auto& e2 = vertices_[t.idx2] - vertices_[t.idx0];
		return normal(cross(e1, e2));
	}

    Vector3f getShadingNormal(int32_t triangleIdx, float u, float v) const
    {
        return
            (normals_[triangles_[triangleIdx].idx0] * (1.0f - u - v) +
            normals_[triangles_[triangleIdx].idx1] * v +
            normals_[triangles_[triangleIdx].idx2] * u);
    }

	inline int32_t triangleCount() const
	{
		return static_cast<int32_t>(triangles_.size());
	}

    inline Bsdf* getBsdf() const
    {
        return bsdf_.get();
    }

	const std::vector<Triangle>& getTriangles() const
	{
		return triangles_;
	}

	const std::vector<Vector3f> getVertices() const
	{
		return vertices_;
	}

private:

    std::vector<Vector3f> vertices_;
    std::vector<Vector3f> normals_;
    std::vector<Triangle> triangles_;
    std::shared_ptr<Bsdf> bsdf_;
};

#endif // TRIANGLE_H

