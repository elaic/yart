#include "bvhaccel.h"

#include <cstdint>

#include "bbox.h"

namespace {
    static const uint8_t AXIS_X = 0;
    static const uint8_t AXIS_Y = 1;
    static const uint8_t AXIS_Z = 2;

    enum SplitAxis : uint8_t {
        X = 0,
        Y = 1,
        Z = 2
    };
}

struct BvhBoundsInfo {
    // Maybe this should be const?
    BBox bounds;
    Vector3f center;
    size_t meshId;
    size_t triangleId;

    BvhBoundsInfo(const BBox& bounds, size_t meshId, size_t triangleId)
        : bounds(bounds), meshId(meshId), triangleId(triangleId)
    {
        center = (bounds.min + bounds.max) * 0.5;
    }
};

struct BvhNode {
    size_t    triangleStartOffset;
    size_t    numTriangles;
    BBox      bounds;
    SplitAxis splitAxis;
    BvhNode*  childNodes[2];
};

BvhAccel::BvhAccel(const Scene& scene)
{
    // Fill in the vector with triangle bounding box data
    std::vector<BvhBoundsInfo> buildData;

    const auto& meshes = scene.getTriangleMeshes();

    // Calculate the number of BvhBoundsInfo structs neccessary, to reserve
    // vector space up front
    size_t numTriangles = 0;
    for (const auto& mesh : meshes) {
        numTriangles += mesh.getTriangles().size();
    }
    buildData.reserve(numTriangles);

    // Calculate bounding information for each triangle
    for (size_t mid = 0; mid < meshes.size(); ++mid) {
        const auto& mesh = meshes[mid];
        const auto& vertices = mesh.getVertices();
        const auto& triangles = mesh.getTriangles();

        for (size_t tid = 0; tid < triangles.size(); ++tid) {
            const auto& triangle = triangles[tid];

            // Calculate triangle bounding box
            auto bounds = BBox(vertices[triangle.idx0]);
            bounds = boxUnion(bounds, vertices[triangle.idx1]);
            bounds = boxUnion(bounds, vertices[triangle.idx2]);

            buildData.emplace_back(bounds, mid, tid);
        }
    }

    // Fill in this structure when building bvh tree. After the bvh nodes are
    // generated, optimized triangle representation will be put into another
    // vector, for fast ray triangle intersection tests. The order in which to
    // put the triangles there will be determined using this data
    using MeshTrianglePair = std::tuple<size_t, size_t>;
    std::vector<MeshTrianglePair> triangles;
    triangles.reserve(numTriangles);
}

