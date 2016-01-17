#if !defined(SHAPE_H)
#define SHAPE_H

#include "vector.h"

struct Ray;
struct RayHitInfo;
class Bsdf;
class AreaLight;

class Shape {
public:
    Shape() : light_(nullptr) { }

	virtual bool intersect(const Ray& ray, RayHitInfo* const hitInfo) const = 0;
	virtual Vector3f sample(float u1, float u2, float* pdf) const = 0;
	virtual float area() const = 0;

    void setLight(AreaLight* light)
    {
        light_ = light;
    }

    AreaLight* getLight() const
    {
        return light_;
    }

private:
    AreaLight* light_;
};

#endif // SHAPE_H
