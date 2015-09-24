#if !defined(SCENE_H)
#define SCENE_H

#include <vector>

#include "vector.h"

#include "sphere.h"
#include "triangle.h"

class Scene {
public:
	Scene(
		const std::vector<TriangleMesh> meshes, 
		const std::vector<Sphere> spheres)
		: meshes_(meshes)
		, spheres_(spheres)
	{ }

	bool intersect(const Ray& ray, RayHitInfo* const isect) const
	{
		static const float inf = 1e20f;
		float d;
		isect->t = inf;
		// not sure if this is safe, but probably better than uninitialized
		int id = -1;
		for (auto i = 0; i < spheres_.size(); ++i) {
			d = spheres_[i].intersect(ray);
			if (d < isect->t) {
				isect->t = d;
				id = i;
			}
		}
		if (isect->t < inf) {
			isect->normal = normal((ray.orig + isect->t * ray.dir) -
				spheres_[id].position);
			isect->bsdf = spheres_[id].bsdf.get();
		}

		RayHitInfo hitInfo;
		for (const auto& mesh : meshes_) {
			if (mesh.intersect(ray, &hitInfo) && hitInfo.t < isect->t) {
				*isect = hitInfo;
			}
		}

		return isect->t < inf;
	}

	bool intersectShadow(const Ray& ray, float maxT) const
	{
		float d;
		// not sure if this is safe, but probably better than uninitialized
		int id = -1;
		for (auto i = 0; i < spheres_.size(); ++i) {
			d = spheres_[i].intersect(ray);
			if (d < maxT) {
				return true;
			}
		}

		RayHitInfo hitInfo;
		for (const auto& mesh : meshes_) {
			if (mesh.intersect(ray, &hitInfo) && hitInfo.t < maxT) {
				return true;
			}
		}

		return false;
	}

	static Scene makeCornellBox();

private:
	std::vector<TriangleMesh> meshes_;
	std::vector<Sphere> spheres_;
};

#endif // SCENE_H
