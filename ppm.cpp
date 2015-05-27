#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <type_traits>

#include "bitmap.h"
#include "constants.h"
#include "frame.h"
#include "rng.h"
#include "vector.h"

#include "bsdf.h"
#include "light.h"

namespace {
	//static const float ALPHA = 0.7f;
}

int primes[61] = {
	2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71,
	73, 79, 83, 89, 97, 101, 103, 107, 109, 113, 127, 131, 137, 139, 149, 151,
	157, 163, 167, 173, 179, 181, 191, 193, 197, 199, 211, 223, 227, 229, 233,
	239, 241, 251, 263, 269, 271, 277, 281, 283
};

inline int rev(const int i, const int p)
{
	if (i == 0)
		return i;
	else
		return p - i;
}

float halton(const int b, int j)
{
	const auto prime = primes[b];
	float h = 0.0f;
	float f = 1.0f / static_cast<float>(prime);
	float fct = f;

	while (j > 0) {
		h += rev(j % prime, prime) * fct;
		j /= prime;
		fct *= f;
	}

	return h;
}

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

struct AABB {
	Vector3f min;
	Vector3f max;

	inline void fit(const Vector3f& point)
	{
		min.x = std::min(min.x, point.x);
		min.y = std::min(min.y, point.y);
		min.z = std::min(min.z, point.z);

		max.x = std::max(max.x, point.x);
		max.y = std::max(max.y, point.y);
		max.z = std::max(max.z, point.z);
	}

	void reset()
	{
		min = Vector3f(1e20f, 1e20f, 1e20f);
		max = Vector3f(-1e20f, -1e20f, -1e20f);
	}
};

struct HitInfo {
	Vector3f f;
	Vector3f position;
	Vector3f normal;
	Vector3f flux;

	float r2;
	uint32_t n;
	int32_t pix;
};

uint32_t hashCount;
uint32_t pixelIdx;
uint32_t pixelCount;
float hashS;

using HitList = std::vector<HitInfo>;
using HashGrid = std::vector<std::vector<HitInfo>>;
HitList hitPoints;
HashGrid hashGrid;

inline uint32_t hash(const Vector3i& gridIdx)
{
	return static_cast<uint32_t>(
		(gridIdx.x * 73856093) ^ (gridIdx.y * 19349663) ^ (gridIdx.z * 83492791)) % hashCount;
}

void buildHashGrid(const int w, const int h)
{
	using irange = range<int32_t>;
	AABB hpbbox;
	hpbbox.reset();

	for (const auto& hitInfo : hitPoints) {
		hpbbox.fit(hitInfo.position);
	}

	Vector3f ssize = hpbbox.max - hpbbox.min;
	auto irad = ((ssize.x + ssize.y + ssize.z) / 3.0f) / ((w + h) / 2.0f) * 2.0f;

	hpbbox.reset();
	auto vphoton = 0;
	for (auto& hitInfo : hitPoints) {
		hitInfo.r2 = irad * irad;
		hitInfo.n = 0;
		hitInfo.flux = Vector3f();
		++vphoton;
		hpbbox.fit(hitInfo.position - irad);
		hpbbox.fit(hitInfo.position + irad);
	}
	
	hashS = 1.0f / (irad * 2.0f);
	hashCount = vphoton;

	hashGrid.resize(hashCount);

	for (const auto& hitInfo : hitPoints) {
		Vector3f min = ((hitInfo.position - irad) - hpbbox.min) * hashS;
		Vector3f max = ((hitInfo.position + irad) - hpbbox.min) * hashS;

		auto rangez = irange(std::abs(static_cast<int32_t>(min.z)), 
			std::abs(static_cast<int32_t>(max.z)));
		auto rangey = irange(std::abs(static_cast<int32_t>(min.y)),
			std::abs(static_cast<int32_t>(max.y)));
		auto rangex = irange(std::abs(static_cast<int32_t>(min.x)),
			std::abs(static_cast<int32_t>(max.x)));

		for (auto iz : rangez) {
			for (auto iy : rangey) {
				for (auto ix : rangex) {
					auto hashVal = hash(Vector3i(ix, iy, iz));
					hashGrid[hashVal].push_back(hitInfo);
				}
			}
		}
	}
}

struct Ray {
	Vector3f orig;
	Vector3f dir;

	Ray(Vector3f o, Vector3f d) : orig(o), dir(d) { }
};

enum Bxdf : int32_t {
	Diff,
	Spec,
	Trans,
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
		Vector3f(0.75f, 0.75f, 0.75f), Bxdf::Diff),
	Sphere(1e5f, Vector3f(50.0f, 1e5f, 81.6f),			
		Vector3f(0.75f, 0.75f, 0.75f), Bxdf::Diff),
	Sphere(1e5f, Vector3f(50.0f, -1e5f + 81.6f, 81.6f),	
		Vector3f(0.75f, 0.75f, 0.75f), Bxdf::Diff),
	Sphere(16.5f, Vector3f(27.0f, 16.5f, 47.0f),	
		Vector3f(0.999f, 0.999f, 0.999f), Bxdf::Spec),
	Sphere(16.5f, Vector3f(73.0f, 16.5f, 88.0f),		
		Vector3f(0.999f, 0.999f, 0.999f), Bxdf::Trans),
	Sphere(8.5f, Vector3f(50.0f, 8.5f, 60.0f),			
		Vector3f(0.999f, 0.999f, 0.999f), Bxdf::Diff),
};

int32_t toneMap(float val)
{
	return static_cast<int32_t>(
		std::pow(1.0f - std::exp(-val), 1.0f / 2.2f) * 255.0f + 0.5f
	);
}

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

	return t < inf;
}

auto light = PointLight(
	Vector3f(50.0f, 60.0f, 85.0f), 
	Vector3f(5000.0f, 5000.0f, 5000.0f)
);

void generatePhoton(Ray& pr, Vector3f& flux, int32_t i) 
{
	flux = Vector3f(50000.0f, 50000.0f, 50000.0f) * (PI * 4.0f);
	float p = 2.0f * PI * halton(0, i);
	float t = 2.0f * std::acos(std::sqrt(1.0f - halton(1, i)));
	float st = std::sin(t);
	pr.dir = Vector3f(std::cos(p) * st, std::cos(t), std::sin(p) * st);
	pr.orig = Vector3f(50.0f, 60.0f, 85.0f);
}

Rng rng;

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

void trace(const Ray& ray)
{
	using std::abs;
	using std::cos;
	using std::sin;
	float t;
	int id;

	Spectrum finalColor = Spectrum(0.0f, 0.0f, 0.0f);
	for (int k = 0; k < 5; ++k) {
		Spectrum color = Spectrum(0.0f, 0.0f, 0.0f);
		Vector3f pathWeight = Vector3f(1.0f, 1.0f, 1.0f);
		Ray currentRay = ray;
		id = -1;

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
				Ray lightRay = { intersection + wi * EPS, wi };
				float maxT = length(intersection - light.position());
				if (!traceShadow(lightRay, maxT)) { 
					if (light.isDelta()) {
						Spectrum f = sphere.bsdf->f(wo, wi);
						color = color + (pointwise(pathWeight, 
							pointwise(f, lightEmission)) 
							* (std::abs(dot(nl, wi)) / pdf));
					} else {
						assert(false);
						color = Spectrum(0.0f, 0.0f, 0.0f);
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

				Vector3f dir = hitFrame.toWorld(wi);

				pathWeight = pointwise(pathWeight, refl) * 
					std::abs(dot(dir, nl)) * (1.0f / pdf);
				currentRay = { intersection + dir * EPS, dir };
			}
		}
		finalColor = finalColor + (color * 0.2f);
	}

	HitInfo hi;

	hi.f = finalColor;
	hi.pix = pixelIdx;

	hitPoints.push_back(hi);
}

int main(int /*argc*/, const char* /*argv*/[])
{
	using std::abs;
	int32_t width = 1024;
	int32_t height = 768;

	auto cam = Ray(
		Vector3f(50.0f, 48.0f, 295.6f), 
		Vector3f(0.0f, -0.042612f, -1.0f).normal()
	);
	auto cx = Vector3f(width * 0.5135f / height, 0.0f, 0.0f);
	auto cy = normal(cross(cx, cam.dir)) * 0.5135f;

	auto framebuffer = Bitmap(width, height);
	
	for (int32_t i = 0; i < height; ++i) {
		for (int32_t j = 0; j < width; ++j) {
			pixelIdx = j + i * width;
			Vector3f d = 
				cx * ((j + 0.5f) / width - 0.5f) + 
				cy * (-(i + 0.5f) / height + 0.5f) + cam.dir;
			Ray r = { cam.orig + d * 140.0f, normal(d) };
			trace(r);
		}
	}

	for (const auto& hp : hitPoints) {
		int y = hp.pix / width;
		int x = hp.pix - y * width;
		framebuffer.set(x, y, hp.f);
	}

	framebuffer.write("image.bmp");

	return 0;
}
