#include "scene.h"


Scene Scene::makeCornellBox()
{
	using ShapeList = std::vector<std::shared_ptr<Shape>>;
	ShapeList shapes = {
		std::make_shared<Sphere>(16.5f, Vector3f(27.0f, 16.5f, 47.0f),
			Spectrum(0.999f, 0.999f, 0.999f), Bxdf::FresSpec),
		std::make_shared<Sphere>(16.5f, Vector3f(73.0f, 16.5f, 88.0f),
			Spectrum(0.999f, 0.999f, 0.999f), Bxdf::FresTran),
		std::make_shared<Sphere>(8.5f, Vector3f(50.0f, 8.5f, 60.0f),
			Spectrum(0.999f, 0.999f, 0.999f), Bxdf::TorranceSparrow),
		std::make_shared<Sphere>(2.0f, Vector3f(50.0f, 60.0f, 85.0f),
			Spectrum(0.0f, 0.0f, 0.0f), Bxdf::None),
	};

	using MeshList = std::vector<TriangleMesh>;
	MeshList meshes {
		// Left wall
		TriangleMesh(
		{
			Vector3f(0.0f, 0.0f, 0.0f),
			Vector3f(0.0f, 0.0f, 230.0f),
			Vector3f(0.0f, 80.0f, 0.0f),
			Vector3f(0.0f, 80.0f, 230.0f),
		},
		{
			Triangle(0, 1, 2),
			Triangle(3, 2, 1),
		},
		std::make_shared<Lambertian>(Spectrum(0.75, 0.25, 0.25))
		),

		// Right wall
		TriangleMesh(
		{
			Vector3f(100.0f, 0.0f, 0.0f),
			Vector3f(100.0f, 0.0f, 230.0f),
			Vector3f(100.0f, 80.0f, 0.0f),
			Vector3f(100.0f, 80.0f, 230.0f),
		},
		{
			Triangle(0, 2, 1),
			Triangle(3, 1, 2),
		},
		std::make_shared<Lambertian>(Spectrum(0.25, 0.25, 0.75))
		),

		// Front wall
		TriangleMesh(
		{
			Vector3f(0.0f, 0.0f, 0.0f),
			Vector3f(100.0f, 0.0f, 0.0f),
			Vector3f(0.0f, 80.0f, 0.0f),
			Vector3f(100.0f, 80.0f, 0.0f),
		},
		{
			Triangle(0, 1, 2),
			Triangle(3, 2, 1),
		},
		std::make_shared<Lambertian>(Spectrum(0.75, 0.75, 0.75))
		),

		// Back wall
		TriangleMesh(
		{
			Vector3f(0.0f, 0.0f, 230.0f),
			Vector3f(100.0f, 0.0f, 230.0f),
			Vector3f(0.0f, 80.0f, 230.0f),
			Vector3f(100.0f, 80.0f, 230.0f),
		},
		{
			Triangle(0, 2, 1),
			Triangle(3, 1, 2),
		},
		std::make_shared<Lambertian>(Spectrum(0.25, 0.75, 0.75))
		),

		// Floor wall
		TriangleMesh(
		{
			Vector3f(0.0f, 0.0f, 230.0f),
			Vector3f(100.0f, 0.0f, 230.0f),
			Vector3f(0.0f, 0.0f, 0.0f),
			Vector3f(100.0f, 0.0f, 0.0f),
		},
		{
			Triangle(0, 1, 2),
			Triangle(3, 2, 1),
		},
		std::make_shared<Lambertian>(Spectrum(0.75, 0.75, 0.75))
		),

		// Ceiling wall
		TriangleMesh(
		{
			Vector3f(0.0f, 80.0f, 230.0f),
			Vector3f(100.0f, 80.0f, 230.0f),
			Vector3f(0.0f, 80.0f, 0.0f),
			Vector3f(100.0f, 80.0f, 0.0f),
		},
		{
			Triangle(0, 2, 1),
			Triangle(3, 1, 2),
		},
		std::make_shared<Lambertian>(Spectrum(0.75, 0.75, 0.75))
		),
		// Reflective cube
		TriangleMesh(
		{
			Vector3f(10.0f, 20.0f, 80.0f), // 0
			Vector3f(10.0f, 40.0f, 80.0f), // 1
			Vector3f(30.0f, 20.0f, 80.0f), // 2
			Vector3f(30.0f, 40.0f, 80.0f), // 3
			Vector3f(10.0f, 20.0f, 100.0f), // 4
			Vector3f(10.0f, 40.0f, 100.0f), // 5
			Vector3f(30.0f, 20.0f, 100.0f), // 6
			Vector3f(30.0f, 40.0f, 100.0f), // 7
		},
		{
			Triangle(0, 1, 2), // rear
			Triangle(1, 3, 2),
			Triangle(0, 4, 1), // left
			Triangle(4, 5, 1),
			Triangle(4, 7, 5), // front
			Triangle(7, 4, 6),
			Triangle(7, 6, 3), // right
			Triangle(6, 2, 3),
			Triangle(1, 5, 7), // top
			Triangle(1, 7, 3),
			Triangle(0, 6, 4), // bottom
			Triangle(0, 2, 6),
		},
		//std::make_shared<Lambertian>(Spectrum(0.1f, 0.3f, 0.7f))
		std::make_shared<FresnelConductor>(Spectrum(0.999f, 0.999f, 0.999f),
			Spectrum(0.16f, 0.55f, 1.75f), Spectrum(4.6f, 2.2f, 1.9f))
		),
	};

	using LightList = std::vector<std::shared_ptr<Light>>;
	LightList lights = {
		std::make_shared<PointLight>(
			Vector3f(80.0f, 60.0f, 85.0f),
			Spectrum(700.0f, 700.0f, 700.0f)
		),
		std::make_shared<AreaLight>(
			shapes[shapes.size() - 1],
			Spectrum(500.0f, 500.0f, 500.0f)
		),
	};

    shapes[shapes.size() - 1]->setLight((AreaLight*)lights[lights.size() - 1].get());

	return Scene(meshes, shapes, lights);
}

