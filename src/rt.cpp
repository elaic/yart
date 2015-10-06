#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <type_traits>
#include <vector>

#include "bitmap.h"
#include "camera.h"
#include "constants.h"
#include "frame.h"
#include "rng.h"
#include "semaphore.h"
#include "timer.h"
#include "vector.h"

#include "light.h"
#include "scene.h"
#include "spectrum.h"
#include "sphere.h"
#include "triangle.h"

auto scene_ = Scene::makeCornellBox();

auto light = PointLight(
	Vector3f(50.0f, 60.0f, 85.0f),
	Spectrum(5000.0f, 5000.0f, 5000.0f)
);

int32_t width = 1031;
int32_t height = 775;

auto camera_ = Camera(
    Vector3f(50.0f, 48.0f, 220.0f),
    normal(Vector3f(0.0f, -0.042612f, -1.0f)),
    width,
    height,
	0.785398f
);

auto framebuffer = Bitmap(width, height);
auto framebufferSecondary = Bitmap(width, height);

struct Tile {
    Vector2i start;
    Vector2i end;

    Tile() { }
    Tile(Vector2i start, Vector2i end) : start(start), end(end) { }
};

void trace(int pixelIdx, int32_t x, int32_t y)
{
	using std::abs;

	Rng rng(pixelIdx);
	auto finalColor = Spectrum(0.0f);
	auto finalColorSecondary = Spectrum(0.0f);
    RayHitInfo isect;
    static const int maxIter = 32;
    static const float invMaxIter = 1.0f / maxIter;
	for (int k = 0; k < maxIter; ++k) {
		Spectrum color { 0.0f };
		Spectrum secondaryColor { 0.0f };
		Spectrum pathWeight { 1.0f };

        auto currentRay = camera_.sample(
            x + rng.randomFloat() - 0.5f,
            y + rng.randomFloat() - 0.5f
        );

		for (int i = 0; i < 5; ++i) {
			if (!scene_.intersect(currentRay, &isect))
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
			Spectrum lightEmission = light.sample(intersection, &wi, &pdf);
			auto lightRay = Ray(intersection + wi * EPS, wi);
			lightRay.maxT = length(intersection - light.position());
			if (!scene_.intersectShadow(lightRay)) {
				if (light.isDelta()) {
					Spectrum f = isect.bsdf->f(wo, wi);
					color = color + ((pathWeight * (f * lightEmission))
						* (abs(dot(nl, wi)) / pdf));
					if (i > 0) {
						secondaryColor = 
							secondaryColor + (pathWeight * f * lightEmission
							* (abs(dot(nl, wi)) / pdf));
					}
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
		finalColorSecondary = finalColorSecondary + (secondaryColor * invMaxIter);
	}

    framebuffer.set(x, y, finalColor.toRGB());
	framebufferSecondary.set(x, y, finalColorSecondary.toRGB());
}

void processTile(Tile tile)
{
    for (int32_t y = tile.start.y; y < tile.end.y; ++y) {
        for (int32_t x = tile.start.x; x < tile.end.x; ++x) {
            trace(x + y * width, x, y);
        }
    }
}

std::vector<std::thread> workers;

using WorkQueue = std::vector<Tile>;
WorkQueue workQueue;

using LockGuard = std::unique_lock<std::mutex>;
std::mutex queueMutex;
std::mutex runMutex;
std::condition_variable runCondition;
semaphore taskSemahore(0);

size_t numUnfinished = 0;

static const int32_t numWorkers = 8;

void enqueuTiles(const WorkQueue& tasks)
{
    {
        LockGuard lock(queueMutex);
        workQueue.insert(end(workQueue), begin(tasks), end(tasks));
        printf("Done enqueueing tasks\n");
    }
    {
        LockGuard lock(runMutex);
        numUnfinished = workQueue.size();
    }
}

void runTiles()
{
    printf("Running tasks\n");
    auto size = workQueue.size();
    while (size-- > 0) {
        taskSemahore.post();
    }
}

void tileTask()
{
    for (;;) {
        taskSemahore.wait();

        Tile currentTile;
        {
            LockGuard lock(queueMutex);
            if (workQueue.size() == 0)
                break;
            currentTile = workQueue.back();
            workQueue.pop_back();
        }

        processTile(currentTile);

        {
            LockGuard lock(runMutex);
            auto unfinished = --numUnfinished;
            if (unfinished <= 0)
            {
                runCondition.notify_one();
                printf("tasks finished\n");
                break;
            }
        }
    }
}

void waitForCompletion()
{
    printf("Wait for task completion\n");
    LockGuard lock(runMutex);
    while (numUnfinished > 0)
        runCondition.wait(lock);
}

void workQueueInit()
{
    printf("Init work queue\n");
    workers.reserve(numWorkers);
    for (int32_t i = 0; i < numWorkers; ++i) {
        workers.push_back(std::thread(tileTask));
    }
}

void workQueueShutdown()
{
    printf("Shutdown work queue\n");
    waitForCompletion();
    for (int32_t i = 0; i < numWorkers; ++i) {
        taskSemahore.post();
    }
    std::for_each(begin(workers), end(workers), [&](auto& t){ t.join(); });
}

class Renderer {
public:
	void render(const Scene& /*scene*/, const Camera& camera) const
	{
		Vector2i numFullTiles;
		numFullTiles.x = camera.getWidth() / tileSize_;
		numFullTiles.y = camera.getHeight() / tileSize_;

		std::vector<Tile> tiles;
		tiles.reserve(numFullTiles.x * numFullTiles.y);

		Vector2i start;
		Vector2i end;

		for (auto i = 0; i < numFullTiles.x; ++i) {
			for (auto j = 0; j < numFullTiles.y; ++j) {
				start.x = i * tileSize_;
				start.y = j * tileSize_;
				end.x = (i + 1) * tileSize_;
				end.y = (j + 1) * tileSize_;
				tiles.push_back(Tile(start, end));
			}
		}

		auto leftoverWidth = camera.getWidth() - numFullTiles.x * tileSize_;
		if (leftoverWidth > 0) {
			for (auto i = 0; i < numFullTiles.y; ++i) {
				start.x = numFullTiles.x * tileSize_;
				start.y = i * tileSize_;
				end.x = camera.getWidth();
				end.y = (i + 1) * tileSize_;
				tiles.push_back(Tile(start, end));
			}
		}

		auto leftoverHeight = camera.getHeight() - numFullTiles.y * tileSize_;
		if (leftoverHeight > 0) {
			for (auto i = 0; i < numFullTiles.x; ++i) {
				start.x = i * tileSize_;
				start.y = numFullTiles.y * tileSize_;
				end.x = (i + 1) * tileSize_;
				end.y = camera.getHeight();
				tiles.push_back(Tile(start, end));
			}
		}

		if (leftoverWidth > 0 && leftoverHeight > 0) {
			tiles.push_back(Tile(
				Vector2i(numFullTiles.x * tileSize_, numFullTiles.y * tileSize_),
				Vector2i(camera.getWidth(), camera.getHeight())
			));
		}

		enqueuTiles(tiles);
		runTiles();
		waitForCompletion();
	}

private:
	static constexpr int32_t tileSize_ = 32;
};

int main(int /*argc*/, const char* /*argv*/[])
{
	Renderer renderer;

    workQueueInit();
	scene_.preprocess();

	Timer timer;
	timer.start();
	renderer.render(scene_, camera_);
	auto elapsed = timer.elapsed();

	auto nanosec = elapsed.count();
	auto minutes = nanosec / 60000000000;

	nanosec = nanosec - minutes * 60000000000;
	auto seconds = nanosec / 1000000000;

	nanosec = nanosec - seconds * 1000000000;
	auto milisec = nanosec / 1000000;

	printf("Time spent rendering: %lldm %llds %lldms\n", minutes, seconds, milisec);

	framebuffer.write("image.bmp");
	framebufferSecondary.write("image-secondary.bmp");

	workQueueShutdown();

	int a;
	std::cin >> a;

	return 0;
}
