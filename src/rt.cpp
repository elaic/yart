#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "frame.h"
#include "rng.h"
#include "scheduler.h"
#include "timer.h"

#include "camera.h"
#include "light.h"
#include "scene.h"
#include "spectrum.h"

struct Tile {
    Vector2i start;
    Vector2i end;

    Tile() { }
    Tile(Vector2i start, Vector2i end) : start(start), end(end) { }
};

void trace(const Scene& scene, Camera& camera,
	int pixelIdx, int32_t x, int32_t y)
{
	using std::abs;

	Rng rng(pixelIdx);
	auto finalColor = Spectrum(0.0f);
    RayHitInfo isect;
    static constexpr int maxIter = 32;
    static const float invMaxIter = 1.0f / maxIter;
	/*
	 * This loop should be part of renderer task (concern). It only samples new
	 * direction and gives it to integrator. Perhaps it can find the first
	 * intersection...
	 */
	for (int k = 0; k < maxIter; ++k) {
		Spectrum color { 0.0f };
		Spectrum secondaryColor { 0.0f };
		Spectrum pathWeight { 1.0f };

        auto currentRay = camera.sample(
            x + rng.randomFloat() - 0.5f,
            y + rng.randomFloat() - 0.5f
        );

		/*
		 * This loop should be part of integrator, so that its easily seperable
		 * from renderer
		 */
		for (int i = 0; i < 5; ++i) {
			if (!scene.intersect(currentRay, &isect))
				break;

			/*
			 * variables used below:
			 * wi - incident direction
			 * wo - outgoing direction
			 */
			Vector3f intersection = currentRay.orig + (currentRay.dir * isect.t);
			Vector3f norm = isect.normal;
			Vector3f nl = dot(norm, currentRay.dir) < 0.0f ? norm : norm * -1.0f;

			auto hitFrame = Frame(norm);
			const auto wo = hitFrame.toLocal(-currentRay.dir);

			/*
			 * sample lights
			 */
			Vector3f wi;
			float pdf;
			const auto& lights = scene.getLights();
			int32_t numLights = static_cast<int32_t>(lights.size());
			int32_t lightIdx = std::min(
				static_cast<int32_t>(rng.randomFloat() * numLights),
				numLights
			);
			const auto& light = lights[lightIdx];
			Spectrum lightEmission = light.sample(intersection, &wi, &pdf);
			auto lightRay = Ray(intersection + wi * EPS, wi);
			lightRay.maxT = length(intersection - light.position());
			if (!scene.intersectShadow(lightRay)) {
				if (light.isDelta()) {
					Spectrum f = isect.bsdf->f(wo, wi);
					color = color + (pathWeight * f * lightEmission
						* (abs(dot(nl, wi)) / pdf) * (float)numLights);
				} else {
					assert(false);
					color = Spectrum(0.0f);
				}
			}

			/*
             * continue tracing
             */
			float continueProbability = pathWeight.y();
			if (rng.randomFloat() > continueProbability)
				break;

			pathWeight /= continueProbability;
			Spectrum refl = isect.bsdf->sample(wo, &wi, rng.randomFloat(),
				rng.randomFloat(), &pdf);

			if (refl.y() == 0.0f)
				break;

			Vector3f dir = hitFrame.toWorld(wi);

			pathWeight = pathWeight * refl 
				* abs(dot(dir, nl)) * (1.0f / pdf);
			currentRay = { intersection + dir * EPS, dir };
		}

		finalColor = finalColor + (color * invMaxIter);
	}

    camera.accumulate(x, y, finalColor.toRGB());
}

class TileTask : public Task {
public:
	TileTask(const Tile& tile, const Scene& scene, Camera& camera)
		: tile_(tile)
		, scene_(scene)
		, camera_(camera)
	{ }

	void run() override
	{
		for (int32_t y = tile_.start.y; y < tile_.end.y; ++y) {
			for (int32_t x = tile_.start.x; x < tile_.end.x; ++x) {
				trace(scene_, camera_, x + y * camera_.getWidth(), x, y);
			}
		}
	}

private:
	Tile tile_;
	const Scene& scene_;
	Camera& camera_;
};

class Renderer {
public:
	void render(const Scene& scene, Camera& camera) const
	{
		Vector2i numFullTiles;
		numFullTiles.x = camera.getWidth() / tileSize_;
		numFullTiles.y = camera.getHeight() / tileSize_;

		std::vector<std::unique_ptr<Task>> tiles;
		tiles.reserve(numFullTiles.x * numFullTiles.y);

		Vector2i start;
		Vector2i end;

		for (auto i = 0; i < numFullTiles.x; ++i) {
			for (auto j = 0; j < numFullTiles.y; ++j) {
				start.x = i * tileSize_;
				start.y = j * tileSize_;
				end.x = (i + 1) * tileSize_;
				end.y = (j + 1) * tileSize_;
				tiles.push_back(std::make_unique<TileTask>(
					Tile(start, end), scene, camera
				));
			}
		}

		auto leftoverWidth = camera.getWidth() - numFullTiles.x * tileSize_;
		if (leftoverWidth > 0) {
			for (auto i = 0; i < numFullTiles.y; ++i) {
				start.x = numFullTiles.x * tileSize_;
				start.y = i * tileSize_;
				end.x = camera.getWidth();
				end.y = (i + 1) * tileSize_;
				tiles.push_back(std::make_unique<TileTask>(
					Tile(start, end), scene, camera
				));
			}
		}

		auto leftoverHeight = camera.getHeight() - numFullTiles.y * tileSize_;
		if (leftoverHeight > 0) {
			for (auto i = 0; i < numFullTiles.x; ++i) {
				start.x = i * tileSize_;
				start.y = numFullTiles.y * tileSize_;
				end.x = (i + 1) * tileSize_;
				end.y = camera.getHeight();
				tiles.push_back(std::make_unique<TileTask>(
					Tile(start, end), scene, camera
				));
			}
		}

		if (leftoverWidth > 0 && leftoverHeight > 0) {
			tiles.push_back(std::make_unique<TileTask>(Tile(
				Vector2i(numFullTiles.x * tileSize_, numFullTiles.y * tileSize_),
				Vector2i(camera.getWidth(), camera.getHeight())),
				scene, camera
			));
		}

		enqueuTasks(tiles);
		runTasks();
		waitForCompletion();
	}

private:
	static constexpr int32_t tileSize_ = 32;
};

int main(int /*argc*/, const char* /*argv*/[])
{
	Renderer renderer;

    workQueueInit();

    auto width = 1024;
    auto height = 768;

	auto scene = Scene::makeCornellBox();
	auto camera = Camera(
		Vector3f(50.0f, 48.0f, 220.0f),
		normal(Vector3f(0.0f, -0.042612f, -1.0f)),
		width,
		height,
		0.785398f
	);

	scene.preprocess();

	Timer timer;
	timer.start();
	renderer.render(scene, camera);
	auto elapsed = timer.elapsed();

	auto nanosec = elapsed.count();
	auto minutes = nanosec / 60000000000;

	nanosec = nanosec - minutes * 60000000000;
	auto seconds = nanosec / 1000000000;

	nanosec = nanosec - seconds * 1000000000;
	auto milisec = nanosec / 1000000;

	printf("Time spent rendering: %lldm %llds %lldms\n", minutes, seconds, milisec);

	camera.saveImage("image.bmp");

	workQueueShutdown();

#if defined(_WIN32)
	int a;
	std::cin >> a;
#endif

	return 0;
}

