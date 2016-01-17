#if !defined(SPHERE_H)
#define SPHERE_H

#include <memory>

#include "bsdf.h"
#include "qmc.h"
#include "shape.h"
#include "utils.h"

enum Bxdf : int32_t {
	None,
	Diff,
	Spec,
	Trans,
	FresSpec,
	FresTran,
	TorranceSparrow,
};

class Sphere : public Shape {
public:
	Sphere(float radius, Vector3f position, Spectrum color, Bxdf bxdf)
		: radius_(radius)
		, position_(position)
	{
		switch (bxdf) {
		case Bxdf::None:
			bsdf_ = nullptr;
			break;

		case Bxdf::Diff:
			bsdf_ = std::make_shared<Lambertian>(color);
			break;

		case Bxdf::Spec:
			bsdf_ = std::make_shared<PerfectConductor>(color);
			break;

		case Bxdf::Trans:
			bsdf_ = std::make_shared<PerfectDielectric>(color, 1.33f);
			break;

		case Bxdf::FresSpec:
			bsdf_ = std::make_shared<FresnelConductor>(color,
				Spectrum(0.16f, 0.55f, 1.75f), Spectrum(4.6f, 2.2f, 1.9f));
			break;

		case Bxdf::FresTran:
			bsdf_ = std::make_shared<FresnelDielectric>(color, 1.66f);
			break;

		case Bxdf::TorranceSparrow:
			bsdf_ = std::make_shared<TorranceSparrowConductor>(
				color,
				Spectrum(0.16f, 0.55f, 1.75f),
				Spectrum(4.6f, 2.2f, 1.9f),
				1000.0f);
			break;
		}
	}

	bool intersect(const Ray& ray, RayHitInfo* const hitInfo) const override
	{
		static const double EPS_S = 1e-4f;
		Vector3f op = position_ - ray.orig;
		double b = dot(op, ray.dir);
		double det = b * b - op.length2() + radius_ * radius_;
		if (det < 0)
			return false;
		else
			det = std::sqrt(det);

		hitInfo->u = 0;
		hitInfo->v = 0;
		hitInfo->bsdf = bsdf_.get();
        hitInfo->areaLight = getLight();

		double t = b - det;
		if (t > EPS_S && t < hitInfo->t) {
			hitInfo->t = static_cast<float>(t);
			hitInfo->normal = normal(ray.orig + ray.dir * hitInfo->t - position_);
			return true;
		} else {
			t = b + det;
			if (t > EPS_S && t < hitInfo->t) {
				hitInfo->t = static_cast<float>(t);
				hitInfo->normal = normal(ray.orig + ray.dir * hitInfo->t - position_);
				return true;
			}
		}
		return false;
	}

	Vector3f sample(float u1, float u2, float* pdf) const override
	{
		Vector3f pos = uniformSphereSample(u1, u2) * radius_;
		pos += position_;
		*pdf = uniformSpherePdf();
		return pos;
	}

	float area() const override
	{
		return 4.0f * PI * pow<2>(radius_);
	}

private:
	float radius_;
	Vector3f position_;
	std::shared_ptr<Bsdf> bsdf_;
	
};

#endif // SPHERE_H
