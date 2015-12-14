#if !defined(SHAPE_H)
#define SHAPE_H

#include "vector.h"

struct Ray;
struct RayHitInfo;

class Shape {
public:
	virtual bool intersect(const Ray& ray, RayHitInfo* const hitInfo) const = 0;
	virtual Vector3f sample(float u1, float u2, float* pdf) const = 0;
	virtual float area() const = 0;
};

#endif // SHAPE_H
