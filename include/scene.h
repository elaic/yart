#if !defined(SCENE_H)
#define SCENE_H

#include <vector>

#include "sphere.h"
#include "triangle.h"

class Scene {
public:
	Scene(
		const std::vector<TriangleMesh> meshes, 
		const std::vector<Sphere> spheres)
		: meshes_(meshes)
	{ }

private:
	std::vector<TriangleMesh> meshes_;
	std::vector<Sphere> spheres_;
};

#endif // SCENE_H
