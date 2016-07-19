#include "bvhaccel.h"

#include <cstdint>

#include "bbox.h"
#include "scene.h"

// types, constants and typedefs internal to the file
namespace {

enum SplitAxis : uint8_t {
    X = 0,
    Y = 1,
    Z = 2,
    None,
};

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
    size_t                    triangleStartOffset;
    size_t                    numTriangles;
    BBox                      bounds;
    SplitAxis                 splitAxis;
    std::unique_ptr<BvhNode>  childNodes[2];

    BvhNode(size_t triangleStartOffset, size_t numTriangles, const BBox& bounds)
        : triangleStartOffset(triangleStartOffset)
        , numTriangles(numTriangles)
        , bounds(bounds)
        , splitAxis(SplitAxis::None)
    { }

    BvhNode(SplitAxis splitAxis, const BBox& bounds, std::unique_ptr<BvhNode>&& leftNode,
        std::unique_ptr<BvhNode>&& rightNode)
        : bounds(bounds)
        , splitAxis(splitAxis)
    {
        childNodes[0] = std::move(leftNode);
        childNodes[1] = std::move(rightNode);
    }
};

// Set to 8 which is the width of AVX intructions
static const int32_t minTrianglesInNode = 8;

using MeshTrianglePair = std::tuple<size_t, size_t>;
using BvhBoundsInfoIter = std::vector<BvhBoundsInfo>::iterator;

} // anonymous namespace

struct BvhAccel::FlattenedBvhNode {
    static_assert(sizeof(BBox) == 24, "BBox size != 24 bytes");

    // 2 * Vector3 = 6 * float --- 24 bytes
    BBox bounds;
    // 4 bytes
    union {
        uint32_t childOffset;
        uint32_t triangleOffset;
    };
    // 1 byte
    uint8_t numTriangles;
    // 1 byte
    SplitAxis splitAxis;
    // 30 bytes total
    uint8_t padding[2];

    FlattenedBvhNode(uint32_t triangleStartOffset, uint8_t numTriangles, const BBox& bounds)
        : triangleOffset(triangleStartOffset)
        , bounds(bounds)
        , numTriangles(numTriangles)
        , splitAxis(SplitAxis::None)
    { }

    FlattenedBvhNode(SplitAxis splitAxis, const BBox& bounds, uint32_t childOffset)
        : childOffset(childOffset)
        , bounds(bounds)
        , splitAxis(splitAxis)
    { }
};

static_assert(sizeof(BvhAccel::FlattenedBvhNode), "FlattenedBvhNode size != 32 bytes");

// methods internal to the file
namespace {

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
    auto midpoint = (bbox.max[axis] + bbox.min[axis]) * 0.5;
    auto middle = std::partition(
        begin,
        end,
        [midpoint, axis](const BvhBoundsInfo& bvhBounds) {
        return bvhBounds.center[axis] < midpoint;
    });

    if (middle == begin || middle == end) {
        middle = begin + numTriangles / 2;
        std::nth_element(
            begin,
            middle,
            end,
            [axis](const BvhBoundsInfo& lhs, const BvhBoundsInfo& rhs) {
                return lhs.center[axis] < rhs.center[axis];
            });
    }

    // Create a interior node
    return std::make_unique<BvhNode>(
        axis,
        bbox,
        buildRecursive(begin, middle, triangles),
        buildRecursive(middle, end, triangles));
}

template <bool shadow>
bool traverse(const BvhNode* node, const Ray& ray, const TriAccel* triangles,
    const std::vector<TriangleMesh>& meshes, RayHitInfo* const isect)
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

        auto axis = node->splitAxis;
        leftFirst = ray.dir[axis] < 0;

        if (leftFirst) {
            firstNode = node->childNodes[1].get();
            secondNode = node->childNodes[0].get();
        }

        auto hitLeft = traverse<shadow>(firstNode, ray, triangles, meshes, isect);
        auto hitRight = traverse<shadow>(secondNode, ray, triangles, meshes, isect);

        return hitLeft || hitRight;
    }

    // We are in leaf node
    if (shadow) {
        for (size_t i = 0; i < node->numTriangles; ++i) {
            size_t tri = node->triangleStartOffset + i;
            if (intersect(triangles[tri], ray, isect)) {
                return true;
            }
        }
    } else {
        int triIdx = -1;
        for (size_t i = 0; i < node->numTriangles; ++i) {
            size_t tri = node->triangleStartOffset + i;
            if (intersect(triangles[tri], ray, isect)) {
                // found closest intersection
                triIdx = (int)tri;
            }
        }

        if (triIdx > -1) {
            auto meshIdx = triangles[triIdx].meshIdx;
            auto triangleIdx = triangles[triIdx].triIdx;
            isect->normal = meshes[meshIdx].getNormal(triangleIdx);
            isect->shadingNormal =
                meshes[meshIdx].getShadingNormal(triangleIdx, isect->u, isect->v);
            isect->bsdf = meshes[meshIdx].getBsdf();
            isect->areaLight = nullptr;
            return true;
        }
    }

    return false;
}

void flattenBvhTree(
    std::vector<BvhAccel::FlattenedBvhNode>& flattenedTree,
    const BvhNode* node)
{
    if (node->splitAxis != None) {
        // interior node
        flattenedTree.emplace_back(node->splitAxis, node->bounds, 0);
        auto nodeIdx = flattenedTree.size() - 1;

        flattenBvhTree(flattenedTree, node->childNodes[0].get());
        auto childOffset = flattenedTree.size();
        // TODO: insert asserts before casting
        flattenedTree[nodeIdx].childOffset = (uint32_t)childOffset;
        flattenBvhTree(flattenedTree, node->childNodes[1].get());
    } else {
        // leaf node
        // TODO: insert asserts before casting
        flattenedTree.emplace_back((uint32_t)node->triangleStartOffset, (uint8_t)node->numTriangles, node->bounds);
    }
}

template <bool shadow>
bool traverse(const std::vector<BvhAccel::FlattenedBvhNode>& flattenedTree,
    const Ray& ray, const TriAccel* triangles,
    const std::vector<TriangleMesh>& meshes, RayHitInfo* const isect)
{
    size_t stackOffset = 0;
    // 64 should be enough... Perhaps some restraints should be put in place in
    // building routine
    size_t stack[64];
    size_t currentNode = 0;

    bool hit = false;

    while (true) {
        auto& node = flattenedTree[currentNode];

        if (node.bounds.intersect(ray)) {
            if (node.splitAxis != SplitAxis::None) {
                // internal node
                if (ray.dir[node.splitAxis] > 0) {
                    currentNode = currentNode + 1;
                    stack[stackOffset] = node.childOffset;
                } else {
                    stack[stackOffset] = currentNode + 1;
                    currentNode = node.childOffset;
                }
                stackOffset++;
            } else {
                // leaf node

                if (shadow) {
                    for (size_t i = 0; i < node.numTriangles; ++i) {
                        size_t tri = node.triangleOffset + i;
                        if (intersect(triangles[tri], ray, isect)) {
                            return true;
                        }
                    }
                } else {
                    int triIdx = -1;
                    for (size_t i = 0; i < node.numTriangles; ++i) {
                        size_t tri = node.triangleOffset + i;
                        if (intersect(triangles[tri], ray, isect)) {
                            // found closest intersection
                            triIdx = (int)tri;
                        }
                    }

                    if (triIdx != -1) {
                        hit = true;
                        auto meshIdx = triangles[triIdx].meshIdx;
                        auto triangleIdx = triangles[triIdx].triIdx;
                        isect->normal = meshes[meshIdx].getNormal(triangleIdx);
                        isect->shadingNormal =
                            meshes[meshIdx].getShadingNormal(triangleIdx, isect->u, isect->v);
                        isect->bsdf = meshes[meshIdx].getBsdf();
                        isect->areaLight = nullptr;
                    }
                }

                if (stackOffset == 0) return hit;
                currentNode = stack[--stackOffset];
            }
        } else {
            if (stackOffset == 0) return hit;
            currentNode = stack[--stackOffset];
        }
    }

    return hit;
}

} // anonymous namespace

BvhAccel::BvhAccel(const Scene& scene)
    : scene_(scene)
{
    using std::get;

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

    auto root_ = buildRecursive(buildData.begin(), buildData.end(), triangles);

    triangles_ = alignedAlloc<TriAccel>(numTriangles, 16);
    for (size_t i = 0; i < numTriangles; ++i) {
        MeshTrianglePair tri = triangles[i];
        const auto& m = scene.getTriangleMeshes()[get<0>(tri)];
        const auto& t = m.getTriangles()[get<1>(tri)];
        project(&triangles_[i], t, m.getVertices(), (int32_t)get<1>(tri), (int32_t)get<0>(tri));
    }

    flattenBvhTree(optimizedAccel_, root_.get());
}

BvhAccel::~BvhAccel()
{
    alignedFree(triangles_);
}

bool BvhAccel::intersect(const Ray& ray, RayHitInfo* const isect) const
{
    return traverse<false>(optimizedAccel_, ray, triangles_, scene_.getTriangleMeshes(), isect);
}

bool BvhAccel::intersectShadow(const Ray& ray) const
{
    RayHitInfo isect;
    isect.t = ray.maxT;

    return traverse<true>(optimizedAccel_, ray, triangles_, scene_.getTriangleMeshes(), &isect);
}

