#if !defined(BSDF_H)
#define BSDF_H

#include <algorithm>
#include <cstdio>
#include <memory>

#include "constants.h"
#include "spectrum.h"
#include "vector.h"

template <typename T>
T clamp(T val, T min, T max)
{
	return std::min(std::max(val, min), max);
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

class MicrofacetDistribution {
public:
	virtual float d(const Vector3f& wh) const = 0;
	virtual void sample(const Vector3f& wo, Vector3f* wi,
		float u1, float u2, float* pdf) const = 0;
};

class Blinn : public MicrofacetDistribution {
public:
	Blinn(float exponent)
	{
		exponent_ = std::min(exponent, 10000.0f);
	}

	virtual float d(const Vector3f& wh) const override;

	virtual void sample(const Vector3f& wo, Vector3f* wi, float u1,
		float u2, float* pdf) const override;

private:
	float exponent_;
};

class Bsdf {
public:
	virtual Spectrum f(const Vector3f& wo, const Vector3f& wi) const = 0;
	virtual Spectrum sample(const Vector3f& wo, Vector3f* wi,
		float u1, float u2, float* pdf) const = 0;
	virtual bool isDelta() const = 0;
};

class Lambertian : public Bsdf {
public:
	Lambertian(const Spectrum& reflectance) : reflectance_(reflectance)
	{ }

	virtual Spectrum f(const Vector3f& wo, const Vector3f& wi) const override;

	virtual Spectrum sample(const Vector3f& wo, Vector3f* wi, float u1,
		float u2, float* pdf) const override;

	virtual bool isDelta() const override
	{
		return false;
	}

private:
	Spectrum reflectance_;
};

class PerfectConductor : public Bsdf {
public:
	PerfectConductor(const Spectrum& reflectance) : reflectance_(reflectance)
	{ }

	virtual Spectrum f(const Vector3f& wo, const Vector3f& wi) const override;

	virtual Spectrum sample(const Vector3f& wo, Vector3f* wi, float u1,
		float u2, float* pdf) const override;

	virtual bool isDelta() const override
	{
		return true;
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

	virtual Spectrum f(const Vector3f& wo, const Vector3f& wi) const override;

	virtual Spectrum sample(const Vector3f& wo, Vector3f* wi, float u1,
		float u2, float* pdf) const override;

	virtual bool isDelta() const override
	{
		return true;
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

	virtual Spectrum f(const Vector3f& wo, const Vector3f& wi) const override;

	virtual Spectrum sample(const Vector3f& wo, Vector3f* wi, float u1,
		float u2, float* pdf) const override;

	virtual bool isDelta() const override
	{
		return true;
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

	virtual Spectrum f(const Vector3f& wo, const Vector3f& wi) const override;

	virtual Spectrum sample(const Vector3f& wo, Vector3f* wi, float u1,
		float u2, float* pdf) const override;

	virtual bool isDelta() const override
	{
		return true;
	}

private:
	Spectrum reflectance_;
	float eta_;
};

/*
 * TorranceSparrow microfacet brdf for conductors
 */
class TorranceSparrowConductor : public Bsdf {
public:
	TorranceSparrowConductor(const Spectrum& reflectance,
		const Spectrum& eta, const Spectrum& k, float exponent)
		: reflectance_(reflectance)
		, eta_(eta)
		, k_(k)
	{
		distribution_ = std::make_unique<Blinn>(exponent);
	}

	virtual Spectrum f(const Vector3f& wo, const Vector3f& wi) const override;

	virtual Spectrum sample(const Vector3f& wo, Vector3f* wi,
		float u1, float u2, float* pdf) const override;

	virtual bool isDelta() const override
	{
		return false;
	}

	float G(const Vector3f& wo, const Vector3f& wi, const Vector3f& wh) const;

private:
	Spectrum reflectance_;
	Spectrum eta_;
	Spectrum k_;
	std::unique_ptr<MicrofacetDistribution> distribution_;
};

#endif // BSDF_H
