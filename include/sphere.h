#if !defined(SPHERE_H)
#define SPHERE_H

#include <memory>

#include "bsdf.h"

enum Bxdf : int32_t {
	Diff,
	Spec,
	Trans,
	FresSpec,
	FresTran,
	TorranceSparrow,
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
			bsdf = std::make_shared<FresnelDielectric>(color, 1.66f);
			break;

		case Bxdf::TorranceSparrow:
			bsdf = std::make_shared<TorranceSparrowConductor>(
				color,
				Spectrum(0.16f, 0.55f, 1.75f),
				Spectrum(4.6f, 2.2f, 1.9f),
				1000.0f);
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
			return std::numeric_limits<float>::infinity();
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
				return std::numeric_limits<float>::infinity();
		}
	}
};

#endif // SPHERE_H