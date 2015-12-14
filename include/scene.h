#if !defined(SCENE_H)
#define SCENE_H

#include <vector>

#include "platform.h"
#include "vector.h"

#include "light.h"
#include "sphere.h"
#include "triaccel.h"
#include "triangle.h"

class PointLight;

class Scene {
public:
	Scene(
		const std::vector<TriangleMesh>& meshes, 
		const std::vector<std::shared_ptr<Shape>>& shapes,
		const std::vector<std::shared_ptr<Light>>& lights)
		: meshes_(meshes)
		, shapes_(shapes)
		, lights_(lights)
		, triaccel_(nullptr)
		, triangleCount_(0)
	{ }

	~Scene()
	{
    	FREE_ALIGNED(triaccel_);
	}

	void preprocess()
	{
		triangleCount_ = 0;
		for (const auto& mesh : meshes_) {
			triangleCount_ += mesh.triangleCount();
		}

		// No need to actually free and alloc always, just do it when there is
		// more space required
		FREE_ALIGNED(triaccel_);

        ALLOC_ALIGNED((void**)&triaccel_, 16, triangleCount_ * sizeof(TriAccel));

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
		isect->t = ray.maxT;
		isect->t = std::numeric_limits<float>::infinity();
		isect->areaLight = nullptr;

		for (const auto& shape : shapes_) {
			shape->intersect(ray, isect);
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
		RayHitInfo hitInfo;
		hitInfo.t = ray.maxT;

		for (const auto& shape : shapes_) {
			if (shape->intersect(ray, &hitInfo)) {
				return true;
			}
		}

		for (int32_t i = 0; i < triangleCount_; ++i) {
			if (intersect(triaccel_[i], ray, &hitInfo)) {
				return true;
			}
		}

		return false;
	}

	const std::vector<std::shared_ptr<Light>>& getLights() const
	{
		return lights_;
	}

	static Scene makeCornellBox();

private:
	std::vector<TriangleMesh> meshes_;
	std::vector<std::shared_ptr<Shape>> shapes_;
	std::vector<std::shared_ptr<Light>> lights_;

	TriAccel* triaccel_;
	int32_t triangleCount_;
};

#endif // SCENE_H
