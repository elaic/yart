#include "renderer.h"

#include "frame.h"
#include "platform.h"
#include "rng.h"
#include "scheduler.h"

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

class Integrator {
public:
    virtual ~Integrator() = 0;
    virtual void integrate(const Scene& scene, Camera& camera,
        int32_t x, int32_t y) = 0;

};

FINLINE void trace(const Scene& scene, Camera& camera, int32_t x, int32_t y)
{
	using std::abs;

	Rng rng(y * camera.getWidth() + x);
	auto finalColor = Spectrum(0.0f);
    RayHitInfo isect;
    static constexpr int maxIter = 2;
    static const float invMaxIter = 1.0f / maxIter;
	/*
	 * This loop should be part of renderer task (concern). It only samples new
	 * direction and gives it to integrator. Perhaps it can find the first
	 * intersection...
	 */
	for (int k = 0; k < maxIter; ++k) {
		Spectrum color { 0.0f };
		Spectrum pathWeight { 1.0f };

        auto currentRay = camera.sample(
            x + rng.randomFloat() - 0.5f,
            y + rng.randomFloat() - 0.5f
        );

		bool evaluateDirectLightHit = true;

		/*
		 * This loop should be part of integrator, so that its easily seperable
		 * from renderer
		 */
		for (auto bounce = 0; bounce < 10; ++bounce) {
			if (!scene.intersect8(currentRay, &isect))
				break;

			if (evaluateDirectLightHit && isect.areaLight) {
				color += pathWeight * isect.areaLight->intensity();
			}

			if (!isect.bsdf)
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
				numLights - 1
			);

			const auto& light = lights[lightIdx];
			float eps;
			Vector3f sampledPosition;
			Spectrum lightEmission = light->sample(intersection, &wi, &pdf,
				&sampledPosition, &eps, rng.randomFloat(), rng.randomFloat());

			auto lightRay = Ray(intersection + wi * EPS, wi);
			lightRay.maxT = length(intersection - sampledPosition) - eps;

			if (!scene.intersect8Shadow(lightRay)) {
				Spectrum f = isect.bsdf->f(wo, wi);
				color = color + (pathWeight * f * lightEmission
					* (abs(dot(nl, wi)) / pdf) * (float)numLights);
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

			evaluateDirectLightHit = isect.bsdf->isDelta();

			Vector3f dir = hitFrame.toWorld(wi);

			pathWeight = pathWeight * refl * abs(dot(dir, nl)) / pdf;
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
				trace(scene_, camera_, x, y);
			}
		}
	}

private:
	Tile tile_;
	const Scene& scene_;
	Camera& camera_;
};

void Renderer::render(const Scene& scene, Camera& camera) const
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

