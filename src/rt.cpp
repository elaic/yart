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
	std::shared_ptr<Bsdf> bsdf;

	Sphere(float radius, Vector3f position, Vector3f color, Bxdf bxdf)
		: radius(radius)
		, position(position)
	{
		switch (bxdf) {
			case Bxdf::Diff:
				bsdf = std::make_shared<Lambertian>(color);
				break;

			case Bxdf::Spec:
				bsdf = std::make_shared<PerfectConductor>(color);
				break;

			case Bxdf::Trans:
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
        static const double EPS_S = 0.0f;
		Vector3f op = position - ray.orig;
		double b = dot(op, ray.dir);
		double det = b * b - op.length2() + radius * radius;
		if (det < 0)
			return 1e20f;
		else
			det = std::sqrt(det);

        double t = b - det;
        if (t > EPS_S) {
            return static_cast<float>(t);
        }
        else {
            t = b + det;
            if (t > EPS_S)
                return static_cast<float>(t);
            else
                return 1e20f;
        }
	}
};

using GeometryList = std::vector<Sphere>;
GeometryList geometry = {
	Sphere(1e4f, Vector3f(1e4f + 1.0f, 40.8f, 81.6f),
		Vector3f(0.75f, 0.25f, 0.25f), Bxdf::Diff),
	Sphere(1e4f, Vector3f(-1e4f + 99.0f, 40.8f, 81.6f),
		Vector3f(0.25f, 0.25f, 0.75f), Bxdf::Diff),
	Sphere(1e4f, Vector3f(50.0f, 40.8f, 1e4f),
		Vector3f(0.75f, 0.75f, 0.75f), Bxdf::Diff),
	Sphere(1e4f, Vector3f(50.0f, 40.8f, -1e4f + 170.0f),
		Vector3f(0.25f, 0.75f, 0.25f), Bxdf::Diff),
	Sphere(1e4f, Vector3f(50.0f, 1e4f, 81.6f),
		Vector3f(0.75f, 0.75f, 0.75f), Bxdf::Diff),
	Sphere(1e4f, Vector3f(50.0f, -1e4f + 81.6f, 81.6f),
		Vector3f(0.75f, 0.75f, 0.75f), Bxdf::Diff),
	Sphere(16.5f, Vector3f(27.0f, 16.5f, 47.0f),
		Vector3f(0.999f, 0.999f, 0.999f), Bxdf::FresSpec),
	Sphere(16.5f, Vector3f(73.0f, 16.5f, 88.0f),
		Vector3f(0.999f, 0.999f, 0.999f), Bxdf::FresTran),
	Sphere(8.5f, Vector3f(50.0f, 8.5f, 60.0f),
		Vector3f(0.999f, 0.999f, 0.999f), Bxdf::Diff),
};

auto cube = TriangleMesh(
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
        Triangle(5, 7, 3),
        Triangle(0, 6, 4), // bottom
        Triangle(0, 2, 6),
    },
    std::make_shared<Lambertian>(Spectrum(0.1f, 0.3f, 0.7f))
    //std::make_shared<FresnelDielectric>(Spectrum(0.8f, 0.8f, 0.8f), 1.33f)
);

inline bool intersect(const Ray& ray, RayHitInfo* const isect)
{
	static const float inf = 1e20f;
	float d;
	isect->t = inf;
	// not sure if this is safe, but probably better than uninitialized
    int id = -1;
	for (auto i = 0; i < geometry.size(); ++i) {
		d = geometry[i].intersect(ray);
		if (d < isect->t) {
			isect->t = d;
			id = i;
		}
	}
    if (isect->t < inf) {
        isect->normal = normal((ray.orig + isect->t * ray.dir) -
            geometry[id].position);
        isect->bsdf = geometry[id].bsdf.get();
    }

    RayHitInfo hitInfo;
    if (cube.intersect(ray, &hitInfo) && hitInfo.t < isect->t) {
        *isect = hitInfo;
    }

	return isect->t < inf;
}

auto light = PointLight(
	Vector3f(50.0f, 60.0f, 85.0f),
	Vector3f(5000.0f, 5000.0f, 5000.0f)
);

inline bool traceShadow(const Ray& ray, float maxT)
{
    RayHitInfo localIsect;
	if (intersect(ray, &localIsect) && localIsect.t < maxT)
		return true;

	return false;
}

template <typename T>
static inline bool absEqEps(T value, T comparison, T eps)
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

	Rng rng(pixelIdx);
	auto finalColor = Spectrum(0.0f);
    RayHitInfo isect;
    static const int maxIter = 50;
    static const float invMaxIter = 1.0f / maxIter;
	for (int k = 0; k < maxIter; ++k) {
		Spectrum color = Spectrum(0.0f);
		Vector3f pathWeight = Vector3f(1.0f);

        auto currentRay = camera.sample(
            x + rng.randomFloat() - 0.5f,
            y + rng.randomFloat() - 0.5f
        );

		for (int i = 0; i < 5; ++i) {
			if (!intersect(currentRay, &isect))
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
			{
				Vector3f wi;
				float pdf;
				Spectrum lightEmission = light.sample(intersection, &wi, &pdf);
				auto lightRay = Ray(intersection + wi * EPS, wi);
				float maxT = length(intersection - light.position());
				if (!traceShadow(lightRay, maxT)) {
					if (light.isDelta()) {
						Spectrum f = isect.bsdf->f(wo, wi);
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
				float continueProbability = (pathWeight.x + pathWeight.y + pathWeight.z) / 3;
				if (rng.randomFloat() > continueProbability)
					break;

				pathWeight /= continueProbability;
				Vector3f wi;
				float pdf;
				Spectrum refl = isect.bsdf->sample(wo, &wi, rng.randomFloat(),
					rng.randomFloat(), &pdf);

				if ((refl.x + refl.y + refl.z) / 3.0f == 0.0f)
					break;

				Vector3f dir = hitFrame.toWorld(wi);

				pathWeight = pointwise(pathWeight, refl) *
					std::abs(dot(dir, nl)) * (1.0f / pdf);
				currentRay = { intersection + dir * 1e-2f, dir };
			}
		}
		finalColor = finalColor + (color * invMaxIter);
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

int main(int /*argc*/, const char* /*argv*/[])
{
	using std::abs;
    static const int32_t tileSize = 32;

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
