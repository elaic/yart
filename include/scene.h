#if !defined(SCENE_H)
#define SCENE_H

#include <vector>

#include "vector.h"

#include "sphere.h"
#include "triaccel.h"
#include "triangle.h"

class Scene {
public:
	Scene(
		const std::vector<TriangleMesh> meshes, 
		const std::vector<Sphere> spheres)
		: meshes_(meshes)
		, spheres_(spheres)
		, triaccel_(nullptr)
		, triangleCount_(0)
	{ }

	~Scene()
	{
		_aligned_free(triaccel_);
	}

	void preprocess()
	{
		triangleCount_ = 0;
		for (const auto& mesh : meshes_) {
			triangleCount_ += mesh.triangleCount();
		}

		// No need to actually free and alloc always, just do it when there is
		// more space required
		_aligned_free(triaccel_);

		triaccel_ = (TriAccel*)_aligned_malloc(
			triangleCount_ * sizeof(TriAccel), 16
		);

		auto triaccelIdx = 0;
		for (int meshIdx = 0; meshIdx < meshes_.size(); ++meshIdx) {
			const auto& mesh = meshes_[meshIdx];
			const auto& triangles = mesh.getTriangles();
			for (int triIdx = 0; triIdx < triangles.size(); ++triIdx) {
				const auto& tri = triangles[triIdx];
				project(
					(triaccel_ + triaccelIdx),
					tri,
					mesh.getVertices(),
					triIdx,
					meshIdx
				);
				++triaccelIdx;
			}
		}
	}

	bool intersect(const Ray& ray, RayHitInfo* const isect) const
	{
		using ::intersect;
		float d;
		isect->t = ray.maxT;
		isect->t = std::numeric_limits<float>::infinity();
		// not sure if this is safe, but probably better than uninitialized
		int id = -1;
		for (auto i = 0; i < spheres_.size(); ++i) {
			d = spheres_[i].intersect(ray);
			if (d < isect->t) {
				isect->t = d;
				id = i;
			}
		}

		if (isect->t < ray.maxT) {
			isect->normal = normal((ray.orig + isect->t * ray.dir) -
				spheres_[id].position);
			isect->bsdf = spheres_[id].bsdf.get();
		}

		auto triIdx = -1;
		for (int32_t i = 0; i < triangleCount_; ++i) {
			if (intersect(triaccel_[i], ray, isect)) {
				triIdx = i;
			}
		}

		if (triIdx > -1) {
			auto meshIdx = triaccel_[triIdx].meshIdx;
			auto triangleIdx = triaccel_[triIdx].triIdx;
			isect->normal = meshes_[meshIdx].getNormal(triangleIdx);
			isect->bsdf = meshes_[meshIdx].getBsdf();
		}

		return isect->t < ray.maxT;
	}

	bool intersectShadow(const Ray& ray) const
	{
		using ::intersect;

		float d;
		for (auto i = 0; i < spheres_.size(); ++i) {
			d = spheres_[i].intersect(ray);
			if (d < ray.maxT) {
				return true;
			}
		}

		RayHitInfo hitInfo;
		auto triIdx = -1;
		hitInfo.t = ray.maxT;
		for (int32_t i = 0; i < triangleCount_; ++i) {
			if (intersect(triaccel_[i], ray, &hitInfo)) {
				return true;
			}
		}

		return false;
	}

	static Scene makeCornellBox();

private:
	std::vector<TriangleMesh> meshes_;
	std::vector<Sphere> spheres_;

	TriAccel* triaccel_;
	int32_t triangleCount_;
};

#endif // SCENE_H
