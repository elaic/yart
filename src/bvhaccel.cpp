#include "bvhaccel.h"

#include <cstdint>

#include "bbox.h"

namespace {

// Set to 8 which is the width of AVX intructions
static const int32_t minTrianglesInNode = 8;

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

enum SplitAxis : uint8_t {
    X = 0,
    Y = 1,
    Z = 2,
    None,
};

struct BvhNode {
    size_t                    triangleStartOffset;
    size_t                    numTriangles;
    BBox                      bounds;
    SplitAxis                 splitAxis;
    std::unique_ptr<BvhNode>  childNodes[2];

    BvhNode(size_t triangleStartOffset, size_t numTriangles, BBox bounds)
        : triangleStartOffset(triangleStartOffset)
        , numTriangles(numTriangles)
        , splitAxis(SplitAxis::None)
        , bounds(bounds)
    { }

    BvhNode(SplitAxis splitAxis, BBox bounds, std::unique_ptr<BvhNode>&& leftNode,
        std::unique_ptr<BvhNode>&& rightNode)
        : splitAxis(splitAxis)
        , bounds(bounds)
    {
        childNodes[0] = std::move(leftNode);
        childNodes[1] = std::move(rightNode);
    }
};

using MeshTrianglePair = std::tuple<size_t, size_t>;
using BvhBoundsInfoIter = std::vector<BvhBoundsInfo>::iterator;

std::unique_ptr<BvhNode> buildRecursive(
    BvhBoundsInfoIter begin,
    BvhBoundsInfoIter end,
    std::vector<MeshTrianglePair>& triangles)
{
    auto numTriangles = std::distance(begin, end);
    if (numTriangles < minTrianglesInNode) {
        auto offset = triangles.size();
        BBox bbox;
        for (auto& it = begin; it != end; ++it) {
            bbox = boxUnion(bbox, it->bounds);
            triangles.push_back(MeshTrianglePair(it->meshId, it->triangleId));
        }
        // Create a leaf node
        return std::make_unique<BvhNode>(offset, numTriangles, bbox);
    }

    // Calculate bounding box
    auto iter = begin;
    BBox bbox;
    while (iter != end) {
        bbox = boxUnion(bbox, iter->bounds);
        iter++;
    }

    SplitAxis axis = (SplitAxis)maxExtent(bbox);
    auto midpoint = (bbox.max[axis] - bbox.min[axis]) / 2 + bbox.min[axis];
    auto middle = std::partition(
        begin,
        end,
        [midpoint, axis](const BvhBoundsInfo& bvhBounds) {
        return bvhBounds.center[axis] < midpoint;
    });

    // Create a interior node
    return std::make_unique<BvhNode>(
        axis,
        bbox,
        buildRecursive(begin, middle, triangles),
        buildRecursive(middle, end, triangles));
}

bool traverse(const BvhNode* node, const Ray& ray, const TriAccel* triangles)
{
    // Fast ray rejection
    if (!node->bounds.intersect(ray))
        return false;

    // Check if this is internal node
    if (node->splitAxis != SplitAxis::None) {
        // Determine which subnode to visit first
        auto leftFirst = true;
        auto firstNode = node->childNodes[0].get();
        auto secondNode = node->childNodes[1].get();

        if (!leftFirst) {
            firstNode = node->childNodes[1].get();
            secondNode = node->childNodes[0].get();
        }
        if (traverse(firstNode, ray, triangles))
            return true;

        return traverse(secondNode, ray, triangles);
    }

    // We are in leaf node, check
    RayHitInfo isect;
    isect.t = ray.maxT;
    for (size_t i = 0; i < node->numTriangles; ++i) {
        size_t tri = node->triangleStartOffset + i;
        if (intersect(triangles[tri], ray, &isect)) {
            return true;
        }
    }

    return false;
}

} // anonymous namespace

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
    std::vector<MeshTrianglePair> triangles;
    triangles.reserve(numTriangles);

    auto root = buildRecursive(buildData.begin(), buildData.end(), triangles);
}

