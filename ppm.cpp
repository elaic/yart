#include <algorithm>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <thread>
#include <type_traits>
#include <vector>

#include "bitmap.h"
#include "camera.h"
#include "constants.h"
#include "frame.h"
#include "rng.h"
#include "vector.h"

#include "bsdf.h"
#include "light.h"
#include "triangle.h"

template <typename T>
class range {
	static_assert(std::is_integral<T>::value, "Ranges only work for int types");
public:
	class iterator {
		friend class range;
	public:
		T operator*() const { return i_; }
		const iterator& operator++() { ++i_; return *this; }
		iterator operator++(int) { iterator copy(*this); ++i_; return copy; }

		bool operator==(const iterator& rhs) { return i_ == rhs.i_; }
		bool operator!=(const iterator& rhs) { return i_ != rhs.i_; }

	protected:
		iterator(T i) : i_(i) { }

	private:
		T i_;
	};

	iterator begin() const { return begin_; }
	iterator end() const { return end_; }
	range(T begin, T end) : begin_(begin), end_(end) { }

private:
	iterator begin_;
	iterator end_;
};

template <typename T>
typename range<T>::iterator begin(const range<T>& range)
{
	return range.begin();
}

template <typename T>
typename range<T>::iterator end(const range<T>& range)
{
	return range.end();
}

class semaphore {
public:
    semaphore(int32_t count) : count_(count) { }

    ~semaphore() = default;

    semaphore(const semaphore& copy) = delete;
    semaphore(semaphore&& move) = default;

    semaphore& operator=(const semaphore& copy) = delete;
    semaphore& operator=(semaphore&& move) = default;

    inline void post()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        ++count_;
        condition_.notify_one();
    }

    inline void wait()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        while (count_ == 0)
            condition_.wait(lock);
        --count_;
    }

private:
    int32_t count_;
    std::mutex mutex_;
    std::condition_variable condition_;
};

enum Bxdf : int32_t {
	Diff,
	Spec,
	Trans,
	FresSpec,
	FresTran,
};

struct Sphere {
	float radius;
	Vector3f position;
	// This should be unique_ptr, as the pointer is never shared, but
	// there is a compilation error. Investigate further.
//	std::unique_ptr<Bsdf> bsdf;
	std::shared_ptr<Bsdf> bsdf;

	Sphere(float radius, Vector3f position, Vector3f color, Bxdf bxdf)
		: radius(radius)
		, position(position)
	{
		switch (bxdf) {
			case Bxdf::Diff:
//				bsdf = std::make_unique<Lambertian>(color);
				bsdf = std::make_shared<Lambertian>(color);
				break;

			case Bxdf::Spec:
//				bsdf = std::make_unique<Lambertian>(color);
				bsdf = std::make_shared<PerfectConductor>(color);
				break;

			case Bxdf::Trans:
//				bsdf = std::make_unique<Lambertian>(color);
				bsdf = std::make_shared<PerfectDielectric>(color, 1.33f);
				break;

			case Bxdf::FresSpec:
				bsdf = std::make_shared<FresnelConductor>(color,
					Spectrum(0.16f, 0.55f, 1.75f), Spectrum(4.6f, 2.2f, 1.9f));
				break;

			case Bxdf::FresTran:
				bsdf = std::make_shared<FresnelDielectric>(color, 1.33f);
				break;
		}
	}

	inline float intersect(const Ray& ray) const
	{
		Vector3f op = position - ray.orig;
		float t;
		auto b = op.dot(ray.dir);
		auto det = b * b - op.length2() + radius * radius;
		if (det < 0)
			return 1e20;
		else
			det = std::sqrt(det);

		return (t = b - det) > 1e-4 ? t : ((t = b + det) > 1e-4 ? t : 1e20);
	}
};

using GeometryList = std::vector<Sphere>;
GeometryList geometry = {
	Sphere(1e5f, Vector3f(1e5f + 1.0f, 40.8f, 81.6f),
		Vector3f(0.75f, 0.25f, 0.25f), Bxdf::Diff),
	Sphere(1e5f, Vector3f(-1e5f + 99.0f, 40.8f, 81.6f),
		Vector3f(0.25f, 0.25f, 0.75f), Bxdf::Diff),
	Sphere(1e5f, Vector3f(50.0f, 40.8f, 1e5f),
		Vector3f(0.75f, 0.75f, 0.75f), Bxdf::Diff),
	Sphere(1e5f, Vector3f(50.0f, 40.8f, -1e5f + 170.0f),
		Vector3f(0.25f, 0.75f, 0.25f), Bxdf::Diff),
	Sphere(1e5f, Vector3f(50.0f, 1e5f, 81.6f),
		Vector3f(0.75f, 0.75f, 0.75f), Bxdf::Diff),
	Sphere(1e5f, Vector3f(50.0f, -1e5f + 81.6f, 81.6f),
		Vector3f(0.75f, 0.75f, 0.75f), Bxdf::Diff),
	Sphere(16.5f, Vector3f(27.0f, 16.5f, 47.0f),
		Vector3f(0.999f, 0.999f, 0.999f), Bxdf::FresSpec),
	Sphere(16.5f, Vector3f(73.0f, 16.5f, 88.0f),
		Vector3f(0.999f, 0.999f, 0.999f), Bxdf::FresTran),
	Sphere(8.5f, Vector3f(50.0f, 8.5f, 60.0f),
		Vector3f(0.999f, 0.999f, 0.999f), Bxdf::Diff),
};

auto triangle = TriangleMesh(
    {
        Vector3f(20.0f, 20.0f, 60.0f),
        Vector3f(20.0f, 40.0f, 60.0f),
        Vector3f(40.0f, 20.0f, 60.0f)
    },
    { Triangle(0, 1, 2) },
    std::make_shared<Lambertian>(Spectrum(0.6f, 0.6f, 0.6f))
);

inline bool intersect(const Ray& ray, float& t, int32_t& id)
{
	float d;
	float inf = 1e20f;
	t = inf;
	for (GeometryList::size_type i = 0; i < geometry.size(); ++i) {
		d = geometry[i].intersect(ray);
		if (d < t) {
			t = d;
			id = i;
		}
	}

    RayHitInfo hitInfo;
    if (triangle.intersect(ray, &hitInfo))
        t = inf;

	return t < inf;
}

auto light = PointLight(
	Vector3f(50.0f, 60.0f, 85.0f),
	Vector3f(5000.0f, 5000.0f, 5000.0f)
);

bool traceShadow(const Ray& ray, float maxT)
{
	float t = 1e20f;
	int id = -1;
	if (intersect(ray, t, id) && t < maxT)
		return true;

	return false;
}

template <typename T>
static bool absEqEps(T value, T comparison, T eps)
{
	return std::abs(std::abs(value) - std::abs(comparison)) < eps;
}

int32_t width = 1024;
int32_t height = 768;

auto camera = Camera(
    Vector3f(50.0f, 48.0f, 295.6f),
    normal(Vector3f(0.0f, -0.042612f, -1.0f)),
    width,
    height,
    0.5135f
);

auto framebuffer = Bitmap(width, height);

struct Tile {
    Vector2i start;
    Vector2i end;

    Tile() { }
    Tile(Vector2i start, Vector2i end) : start(start), end(end) { }
};

void trace(int pixelIdx, int32_t x, int32_t y)
{
	using std::abs;
	using std::cos;
	using std::sin;
	float t;
	int id;

	Rng rng(pixelIdx);
	auto finalColor = Spectrum(0.0f);
	for (int k = 0; k < 100; ++k) {
		Spectrum color = Spectrum(0.0f);
		Vector3f pathWeight = Vector3f(1.0f);
		id = -1;

        auto currentRay = camera.sample(
            x + rng.randomFloat() - 0.5,
            y + rng.randomFloat() - 0.5
        );

		for (int i = 0; i < 5; ++i) {
			if (!intersect(currentRay, t, id))
				break;

			/*
			 * variables used below:
			 * wi - incident direction
			 * wo - outgoing direction
			 */
			const Sphere& sphere = geometry[id];
			Vector3f intersection = currentRay.orig + (currentRay.dir * t);
			Vector3f norm = normal(intersection - sphere.position);
			Vector3f nl = dot(norm, currentRay.dir) < 0.0f ? norm : norm * -1.0f;

			auto hitFrame = Frame(norm);
			const auto wo = hitFrame.toLocal(-currentRay.dir);

			/*
			 * sample lights
			 */
			{
				Vector3f wi;
				float pdf;
				Spectrum lightEmission = light.sample(intersection, &wi, &pdf);
				auto lightRay = Ray(intersection + wi * EPS, wi);
				float maxT = length(intersection - light.position());
				if (!traceShadow(lightRay, maxT)) {
					if (light.isDelta()) {
						Spectrum f = sphere.bsdf->f(wo, wi);
						color = color + (pointwise(pathWeight,
							pointwise(f, lightEmission))
							* (std::abs(dot(nl, wi)) / pdf));
					} else {
						assert(false);
						color = Spectrum(0.0f);
					}
				}
			}

			/*
			 * continue tracing
			 */
			{
				Vector3f wi;
				float pdf;
				Spectrum refl = sphere.bsdf->sample(wo, &wi, rng.randomFloat(),
					rng.randomFloat(), &pdf);

				if ((refl.x + refl.y + refl.z) / 3.0f == 0.0f)
					break;

				Vector3f dir = hitFrame.toWorld(wi);

				pathWeight = pointwise(pathWeight, refl) *
					std::abs(dot(dir, nl)) * (1.0f / pdf);
				currentRay = { intersection + dir * EPS, dir };
			}
		}
		finalColor = finalColor + (color * 0.01f);
	}

    framebuffer.set(x, y, finalColor);
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

int32_t numUnfinished = 0;

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
    while (true) {
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
            int unfinished = --numUnfinished;
            //printf("%i\n", unfinished);
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

int main(int /*argc*/, const char* /*argv*/[])
{
	using std::abs;
    static const int32_t tileSize = 32;

    Rng jitter(0);

    Vector2i numFullTiles;
    numFullTiles.x = width / tileSize;
    numFullTiles.y = height / tileSize;

    workQueueInit();

    std::vector<Tile> tiles;
    tiles.reserve(8);
    Vector2i start;
    Vector2i end;
    for (auto i = 0; i < numFullTiles.x; ++i) {
        for (auto j = 0; j < numFullTiles.y; ++j) {
            start.x = i * tileSize;
            start.y = j * tileSize;
            end.x = (i + 1) * tileSize;
            end.y = (j + 1) * tileSize;
            tiles.push_back(Tile(start, end));
        }
    }

    enqueuTiles(tiles);
    runTiles();
    waitForCompletion();
    workQueueShutdown();

	framebuffer.write("image.bmp");

	return 0;
}
