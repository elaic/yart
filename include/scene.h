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
    	alignedFree(triaccel_);
	}

	void preprocess()
	{
		triangleCount_ = 0;
		for (const auto& mesh : meshes_) {
			triangleCount_ += mesh.triangleCount();
		}

		// No need to actually free and alloc always, just do it when there is
		// more space required
		alignedFree(triaccel_);

        triaccel_ = alignedAlloc<TriAccel>(triangleCount_, 16);

		auto triaccelIdx = 0;
		for (mesh_size_t meshIdx = 0; meshIdx < meshes_.size(); ++meshIdx) {
			const auto& mesh = meshes_[meshIdx];
			const auto& triangles = mesh.getTriangles();
			for (TriangleMesh::tri_size_t triIdx = 0; triIdx < triangles.size(); ++triIdx) {
				const auto& tri = triangles[triIdx];
				project(
					(triaccel_ + triaccelIdx),
					tri,
					mesh.getVertices(),
					(int)triIdx,
					(int)meshIdx
				);
				++triaccelIdx;
			}
		}
	}

	bool intersect(const Ray& ray, RayHitInfo* const isect) const
	{
		using ::intersect;
		isect->t = ray.maxT;
		isect->areaLight = nullptr;

		for (const auto& shape : shapes_) {
			shape->intersect(ray, isect);
		}

		auto triIdx = -1;
		for (auto i = 0; i < (int)triangleCount_; ++i) {
			if (intersect(triaccel_[i], ray, isect)) {
				triIdx = i;
			}
		}

		if (triIdx > -1) {
			auto meshIdx = triaccel_[triIdx].meshIdx;
			auto triangleIdx = triaccel_[triIdx].triIdx;
			isect->normal = meshes_[meshIdx].getNormal(triangleIdx);
			isect->bsdf = meshes_[meshIdx].getBsdf();
            isect->areaLight = nullptr;
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

		for (size_t i = 0; i < triangleCount_; ++i) {
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
    using mesh_size_t = std::vector<TriangleMesh>::size_type;
    using shape_size_t = std::vector<Shape>::size_type;
    using light_size_t = std::vector<Light>::size_type;

	std::vector<TriangleMesh> meshes_;
	std::vector<std::shared_ptr<Shape>> shapes_;
	std::vector<std::shared_ptr<Light>> lights_;

	TriAccel* triaccel_;
	size_t triangleCount_;
};

#endif // SCENE_H
