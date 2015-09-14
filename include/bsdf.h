#if !defined(BSDF_H)
#define BSDF_H

#include "constants.h"
#include "vector.h"

#include <cstdio>

namespace {
	static const float COS_EPS = 1e-4f;
}

template <typename T>
T clamp(T val, T min, T max)
{
	return std::min(std::max(val, min), max);
}

void concentricSampleDisk(float u1, float u2, float* dx, float* dy)
{
	using std::cos;
	using std::sin;
	float r, theta;

	float sx = 2 * u1 - 1;
	float sy = 2 * u2 - 1;

	if (sx == 0.0f && sy == 0.0f) {
		*dx = 0.0f;
		*dy = 0.0f;
		return;
	}

	if (sx >= -sy) {
		if (sx > sy) {
			r = sx;
			if (sy > 0.0f)	theta = sy / r;
			else			theta = 8.0f + sy / r;
		} else {
			r = sy;
			theta = 2.0f - sx / r;
		}
	} else {
		if (sx <= sy) {
			r = -sx;
			theta = 4.0f - sy / r;
		} else {
			r = -sy;
			theta = 6.0f - sx / r;
		}
	}

	theta *= PI / 4.0f;

	*dx = r * cos(theta);
	*dy = r * sin(theta);
}

inline Vector3f cosHemisphereSample(float u1, float u2)
{
	using std::sqrt;
	using std::max;
	Vector3f ret;
	concentricSampleDisk(u1, u2, &ret.x, &ret.y);
	ret.z = sqrt(max(0.0f, 1.0f - ret.x * ret.x - ret.y * ret.y));
	return ret;
}

inline float cosHemispherePdf(float cosTheta, float /*phi*/)
{
	return cosTheta * INV_PI;
}

inline Vector3f uniformHemisphereSample(float u1, float u2)
{
	using std::sqrt;
	using std::max;

	float z = u1;
	float r = sqrt(max(0.0f, 1.0f - z * z));
	float phi = 2.0f * PI * u2;
	float x = r * std::cos(phi);
	float y = r * std::sin(phi);

	return Vector3f(x, y, z);
}

inline float cosTheta(const Vector3f& w)
{
	return w.z;
}

inline float absCosTheta(const Vector3f& w)
{
	return std::abs(w.z);
}

inline float sinTheta2(const Vector3f& w)
{
	return std::max(0.0f, 1.0f - cosTheta(w) * cosTheta(w));
}

inline float sinTheta(const Vector3f& w)
{
	return std::sqrt(sinTheta2(w));
}

inline float cosPhi(const Vector3f& w)
{
	float sinTheta = ::sinTheta(w);
	if (sinTheta == 0.0f) return 1.0f;
	return clamp(w.x / sinTheta, -1.0f, 1.0f);
}

inline float sinPhi(const Vector3f& w)
{
	float sinTheta = ::sinTheta(w);
	if (sinTheta == 0.0f) return 0.0f;
	return clamp(w.y / sinTheta, -1.0f, 1.0f);
}

inline bool sameHemisphere(const Vector3f& w, const Vector3f& w1)
{
	return w.z * w1.z > 0.0f;
}

/*
 * cosi	-
 * cost	-
 * etai	-
 * etat	-
 */
inline Spectrum fresnelDielectric(float cosi, float cost, const Spectrum& etai,
	const Spectrum& etat)
{
	Spectrum Rparallel =
		((etat * cosi) - (etai * cost)) /
		((etat * cosi) + (etai * cost));
	Spectrum Rperpendicular =
		((etai * cosi) - (etat * cost)) /
		((etai * cosi) + (etat * cost));
	return (pointwise(Rparallel, Rparallel) +
		pointwise(Rperpendicular, Rperpendicular)) / 2.0f;
}

template <int n>
float pow(float val)
{
	return val * pow<n - 1>(val);
}

template<>
float pow<1>(float val)
{
	return val;
}

inline float fresnelDielectricSchlick(float cosi, float etai, float etat)
{
	float R0 = (etai - etat) / (etai + etat);
	R0 *= R0;

	float Rcos = R0 + (1 - R0) * pow<5>(1 - cosi);
	return Rcos;
}

/*
 * eta	- wavelength dependent index od refraction
 * k	- wavelength dependent absorption coefficient
 */
inline Spectrum fresnelConductor(float cosi, const Spectrum& eta,
	const Spectrum& k)
{
	Spectrum tmp = (pointwise(eta, eta) + pointwise(k, k)) * cosi * cosi;
	Spectrum Rparl2 = (tmp - (2.0f * eta * cosi) + 1.0f) /
		(tmp + (2.0f * eta * cosi) + 1.0f);
	Spectrum tmp2 = pointwise(eta, eta) + pointwise(k, k);
	Spectrum Rperp2 = (tmp2 - (2.0f * eta * cosi) + cosi * cosi) /
		(tmp2 + (2.0f * eta * cosi) + cosi * cosi);
	return (Rparl2 + Rperp2) / 2.0f;
}

class Bsdf {
public:
	virtual Spectrum f(const Vector3f& wo, const Vector3f& wi) const = 0;
	virtual Spectrum sample(const Vector3f& wo, Vector3f* wi,
		float u1, float u2, float* pdf) const = 0;
};

class Lambertian : public Bsdf {
public:
	Lambertian(const Spectrum& reflectance) : reflectance_(reflectance)
	{ }

	virtual Spectrum f(const Vector3f& wo, const Vector3f& wi) const override
	{
		((void)wo);
		((void)wi);
		return reflectance_ * INV_PI;
	}

	virtual Spectrum sample(const Vector3f& wo, Vector3f* wi, float u1,
		float u2, float* pdf) const override
	{
		*wi = cosHemisphereSample(u1, u2);
		if (wo.z < 0.0f) wi->z *= -1.0f;
		*pdf = sameHemisphere(wo, *wi) ? absCosTheta(*wi) * INV_PI : 0.0f;
		if (absCosTheta(*wi) < COS_EPS)
			return Spectrum(0.0f, 0.0f, 0.0f);
		return f(wo, *wi);
	}

private:
	Spectrum reflectance_;
};

class PerfectConductor : public Bsdf {
public:
	PerfectConductor(const Spectrum& reflectance) : reflectance_(reflectance)
	{ }

	virtual Spectrum f(const Vector3f& wo, const Vector3f& wi) const override
	{
		(void(wo));
		(void(wi));
		return Spectrum(0.0f, 0.0f, 0.0f);
	}

	virtual Spectrum sample(const Vector3f& wo, Vector3f* wi, float u1,
		float u2, float* pdf) const override
	{
		(void(u1));
		(void(u2));
		wi->x = -wo.x;
		wi->y = -wo.y;
		wi->z = wo.z;
		*pdf = 1.0f;
		if (absCosTheta(*wi) < COS_EPS)
			return Spectrum(0.0f, 0.0f, 0.0f);
		return reflectance_ / absCosTheta(*wi);
	}

private:
	Spectrum reflectance_;
};

class PerfectDielectric : public Bsdf {
public:
	PerfectDielectric(const Spectrum& reflectance, float eta)
		: reflectance_(reflectance)
		, eta_(eta)
	{ }

	virtual Spectrum f(const Vector3f& wo, const Vector3f& wi) const override
	{
		(void(wo));
		(void(wi));
		return Spectrum(0.0f, 0.0f, 0.0f);
	}

	virtual Spectrum sample(const Vector3f& wo, Vector3f* wi, float u1,
		float u2, float* pdf) const override
	{
		(void(u1));
		(void(u2));
		bool entering = cosTheta(wo) > 0;
		float eta = entering ? 1.0f / eta_ : eta_;
		float sini2 = sinTheta2(wo);
		float sint2 = eta * eta * sini2;

		// total internal reflection
		if (sint2 > 1.0f)
			return Spectrum(0.0f, 0.0f, 0.0f);

		float cost = std::sqrt(std::max(0.0f, 1.0f - sint2));
		if (entering) cost = -cost;

		float sintOverSini = eta;
		*wi = Vector3f(sintOverSini * -wo.x, sintOverSini * -wo.y, cost);
		*pdf = 1.0f;
		if (absCosTheta(*wi) < COS_EPS)
			return Spectrum(0.0f, 0.0f, 0.0f);
		return reflectance_ / absCosTheta(*wi);
	}

private:
	Spectrum reflectance_;
	float eta_;
};

class FresnelConductor : public Bsdf {
public:
	FresnelConductor(const Spectrum& reflectance, const Spectrum& eta,
		const Spectrum& k)
		: reflectance_(reflectance)
		, eta_(eta)
		, k_(k)
	{ }

	virtual Spectrum f(const Vector3f& wo, const Vector3f& wi) const override
	{
		(void(wo));
		(void(wi));
		return Spectrum(0.0f);
	}

	virtual Spectrum sample(const Vector3f& wo, Vector3f* wi, float u1,
		float u2, float* pdf) const override
	{
		(void(u1));
		(void(u2));
		wi->x = -wo.x;
		wi->y = -wo.y;
		wi->z = wo.z;
		*pdf = 1.0f;
		if (absCosTheta(*wi) < COS_EPS)
			return Spectrum(0.0f);
		return pointwise(
				fresnelConductor(absCosTheta(wo), eta_, k_),
				reflectance_) / absCosTheta(*wi);
	}

private:
	Spectrum reflectance_;
	Spectrum eta_;
	Spectrum k_;
};

class FresnelDielectric : public Bsdf {
public:
	FresnelDielectric(const Spectrum& reflectance, float eta)
		: reflectance_(reflectance)
		, eta_(eta)
	{ }

	virtual Spectrum f(const Vector3f& wo, const Vector3f& wi) const override
	{
		(void(wo));
		(void(wi));
		return Spectrum(0.0f);
	}

	virtual Spectrum sample(const Vector3f& wo, Vector3f* wi, float u1,
		float u2, float* pdf) const override
	{
		(void(u2));
		bool entering = cosTheta(wo) > 0;
		float eta = entering ? 1.0f / eta_ : eta_;
		float sini2 = sinTheta2(wo);
		float sint2 = eta * eta * sini2;

		// total internal reflection, reflect ray
		if (sint2 > 1.0f) {
			*wi = Vector3f(-wo.x, -wo.y, wo.z);
			if (absCosTheta(*wi) < COS_EPS)
				return Spectrum(0.0f);
			return reflectance_ / absCosTheta(*wi);
		}

		float etai = 1.0f;
		float etat = eta_;
		if (!entering)
			std::swap(etai, etat);

		float cost = std::max(0.0f, std::sqrt(1.0f - sint2));
		Spectrum fresnel = fresnelDielectric(absCosTheta(wo), cost,
			Spectrum(etai), Spectrum(etat));

		float reflectionProbability = (fresnel.x + fresnel.y + fresnel.z) / 3.0f;

		if (u1 < reflectionProbability) {
			// reflection
			*wi = Vector3f(-wo.x, -wo.y, wo.z);
			*pdf = reflectionProbability;
			if (absCosTheta(*wi) < COS_EPS)
				return Spectrum(0.0f);
			return pointwise(fresnel, reflectance_) / absCosTheta(*wi);
		} else {
			// refraction
			cost = std::sqrt(std::max(0.0f, 1.0f - sint2));
			if (entering) cost = -cost;

			float sintOverSini = eta;
			*wi = Vector3f(sintOverSini * -wo.x, sintOverSini * -wo.y, cost);
			*pdf = 1.0f - reflectionProbability;
			if (absCosTheta(*wi) < COS_EPS)
				return Spectrum(0.0f);
			return pointwise(Spectrum(1.0f) - fresnel, reflectance_) / absCosTheta(*wi);
		}
	}

private:
	Spectrum reflectance_;
	float eta_;
};

#endif // BSDF_H
