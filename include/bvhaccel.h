#if !defined(BVHACCEL_H)
#define BVHACCEL_H

#include "vector.h"
#include "scene.h"

// BVH with binary splits. Algorithm work flow:
// 1. Compute bounds of each triangle in each mesh. Store mesh id and triangle
//    id for in the bounds structure
// 2. Based on bounds, create a binary tree to partition the objects using SAH
// 3. Every time a leaf node is created, record which triangle from which mesh
//    should go into that node in a triangle vector, and just store offset to
//    that vector in BVH node
// 4. After tree is built, process the triangle vector, creating optimized
//    triangle representation using TriAccel
// 5. Create oprimized tree layout and store it in a vector instead of linked
//    tree nodes.
//
// Possibilities for intersection code optimizations:
// 1. Instead of using TriAccel representation, use the TriAccel8 one. This
//    might be a bit more tricky to fill in, but should provide measureable
//    benefit
// 2. Instead of doing 2 way splitting, use vector width way splitting (8 for
//    avx), so that BHV node test can also be done in a vectorized way.
class BvhAccel {
public:
    BvhAccel(const Scene& scene);

    // Copying is expensive and makes little sense. Delete for now.
    BvhAccel(const BvhAccel& copy) = delete;

    // Moving should be fine. Leave as default for now.
    BvhAccel(BvhAccel&& move) = default;

    // Copying is expensive and makes little sense. Delete for now.
    BvhAccel& operator=(const BvhAccel& copy) = delete;

    // Moving should be fine. Leave as default for now.
    BvhAccel& operator=(BvhAccel&& move) = default;

private:
};

#endif // BVHACCEL_H
