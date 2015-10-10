#include "bsdf.h"

#include "qmc.h"

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
		(etat * cosi - etai * cost) /
		(etat * cosi + etai * cost);
	Spectrum Rperpendicular =
		(etai * cosi - etat * cost) /
		(etai * cosi + etat * cost);
	return (Rparallel * Rparallel + Rperpendicular * Rperpendicular) / 2.0f;
}

template <int n>
inline float pow(float val)
{
	return val * pow<n - 1>(val);
}

template<>
inline float pow<1>(float val)
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
	Spectrum tmp = (eta * eta + k * k) * cosi * cosi;
	Spectrum Rparl2 = (tmp - (2.0f * eta * cosi) + 1.0f) /
		(tmp + (2.0f * eta * cosi) + 1.0f);
	Spectrum tmp2 = eta * eta + k * k;
	Spectrum Rperp2 = (tmp2 - (2.0f * eta * cosi) + cosi * cosi) /
		(tmp2 + (2.0f * eta * cosi) + cosi * cosi);
	return (Rparl2 + Rperp2) / 2.0f;
}

float Blinn::d(const Vector3f& wh) const
{
    float cosTetaH = absCosTheta(wh);
    return (exponent_ + 2.0f) * INV_2PI * std::pow(cosTetaH, exponent_);
}

void Blinn::sample(const Vector3f& wo, Vector3f* wi, float u1,
    float u2, float* pdf) const
{
    float cosTheta = std::pow(u1, 1.0f / (exponent_ + 1.0f));
    float sinTheta = std::sqrt(std::max(0.0f, 1.0f - cosTheta * cosTheta));
    float phi = u2 * 2.0f * PI;
    Vector3f wh = sphericalDirection(sinTheta, cosTheta, phi);

    if (!sameHemisphere(wo, wh))
        wh = -wh;
    *wi = -wo + 2.0f * dot(wo, wh) * wh;

    float blinnPdf = ((exponent_ + 1.0f) * std::pow(cosTheta, exponent_)) /
        (2.0f * PI * 4.0f * dot(wo, wh));
    if (dot(wo, wh) < 0.0f) 
        blinnPdf = 0.0f;

    *pdf = blinnPdf;
}

/*
 * Lambertian
 */
Spectrum Lambertian::f(const Vector3f& wo, const Vector3f& wi) const
{
    ((void)wo);
    ((void)wi);
    return reflectance_ * INV_PI;
}

Spectrum Lambertian::sample(const Vector3f& wo, Vector3f* wi, float u1,
    float u2, float* pdf) const
{
    *wi = cosHemisphereSample(u1, u2);
    if (wo.z < 0.0f) wi->z *= -1.0f;
    *pdf = sameHemisphere(wo, *wi) ? absCosTheta(*wi) * INV_PI : 0.0f;
    if (absCosTheta(*wi) < EPS)
        return Spectrum(0.0f, 0.0f, 0.0f);
    return f(wo, *wi);
}

/*
 * Perfect conductor
 */
Spectrum PerfectConductor::f(const Vector3f& wo, const Vector3f& wi) const
{
    (void(wo));
    (void(wi));
    return Spectrum(0.0f, 0.0f, 0.0f);
}

Spectrum PerfectConductor::sample(const Vector3f& wo, Vector3f* wi, float u1,
    float u2, float* pdf) const
{
    (void(u1));
    (void(u2));
    wi->x = -wo.x;
    wi->y = -wo.y;
    wi->z = wo.z;
    *pdf = 1.0f;
    if (absCosTheta(*wi) < EPS)
        return Spectrum(0.0f, 0.0f, 0.0f);
    return reflectance_ / absCosTheta(*wi);
}

/*
 * Perfect dielectric
 */
Spectrum PerfectDielectric::f(const Vector3f& wo, const Vector3f& wi) const
{
    (void(wo));
    (void(wi));
    return Spectrum(0.0f, 0.0f, 0.0f);
}

Spectrum PerfectDielectric::sample(const Vector3f& wo, Vector3f* wi, float u1,
    float u2, float* pdf) const
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
    if (absCosTheta(*wi) < EPS)
        return Spectrum(0.0f, 0.0f, 0.0f);
    return reflectance_ / absCosTheta(*wi);
}

/*
 * Fresnel conductor
 */
Spectrum FresnelConductor::f(const Vector3f& wo, const Vector3f& wi) const
{
    (void(wo));
    (void(wi));
    return Spectrum(0.0f);
}

Spectrum FresnelConductor::sample(const Vector3f& wo, Vector3f* wi, float u1,
    float u2, float* pdf) const
{
    (void(u1));
    (void(u2));
    wi->x = -wo.x;
    wi->y = -wo.y;
    wi->z = wo.z;
    *pdf = 1.0f;
    if (absCosTheta(*wi) < EPS)
        return Spectrum(0.0f);
    return fresnelConductor(absCosTheta(wo), eta_, k_) * reflectance_
        / absCosTheta(*wi);
}

/*
 * Fresnel dielectric
 */
Spectrum FresnelDielectric::f(const Vector3f& wo, const Vector3f& wi) const
{
    (void(wo));
    (void(wi));
    return Spectrum(0.0f);
}

Spectrum FresnelDielectric::sample(const Vector3f& wo, Vector3f* wi, float u1,
    float u2, float* pdf) const
{
    (void(u2));
    bool entering = cosTheta(wo) > 0;
    float eta = entering ? 1.0f / eta_ : eta_;
    float sini2 = sinTheta2(wo);
    float sint2 = eta * eta * sini2;

    // total internal reflection, reflect ray
    if (sint2 > 1.0f) {
        *wi = Vector3f(-wo.x, -wo.y, wo.z);
        if (absCosTheta(*wi) < EPS)
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

    float reflectionProbability = fresnel.y();

    if (u1 < reflectionProbability) {
        // reflection
        *wi = Vector3f(-wo.x, -wo.y, wo.z);
        *pdf = reflectionProbability;
        if (absCosTheta(*wi) < EPS)
            return Spectrum(0.0f);
        return fresnel * reflectance_ / absCosTheta(*wi);
    } else {
        // refraction
        cost = std::sqrt(std::max(0.0f, 1.0f - sint2));
        if (entering) cost = -cost;

        float sintOverSini = eta;
        *wi = Vector3f(sintOverSini * -wo.x, sintOverSini * -wo.y, cost);
        *pdf = 1.0f - reflectionProbability;
        if (absCosTheta(*wi) < EPS)
            return Spectrum(0.0f);
        return (Spectrum(1.0f) - fresnel) * reflectance_ / absCosTheta(*wi);
    }
}


/*
 * TorranceSparrow microfacet brdf for conductors
 */
Spectrum TorranceSparrowConductor::f(const Vector3f& wo, const Vector3f& wi) const
{
    float cosThetaO = absCosTheta(wo);
    float cosThetaI = absCosTheta(wi);
    if (cosThetaI == 0.0f || cosThetaO == 0.0f)
        return Spectrum(0.0f);

    Vector3f wh = normal(wo + wi);
    float cosThetaH = dot(wi, wh);
    Spectrum f = fresnelConductor(cosThetaH, eta_, k_);
    return (reflectance_ * f) * distribution_->d(wh) * G(wo, wi, wh) /
        (4.0f * cosThetaI * cosThetaO);
}

Spectrum TorranceSparrowConductor::sample(const Vector3f& wo, Vector3f* wi,
    float u1, float u2, float* pdf) const
{
    distribution_->sample(wo, wi, u1, u2, pdf);
    if (!sameHemisphere(wo, *wi))
        return Spectrum(0.0f);

    return f(wo, *wi);
}

float TorranceSparrowConductor::G(const Vector3f& wo, const Vector3f& wi,
    const Vector3f& wh) const
{
    float nDotWh = absCosTheta(wh);
    float nDotWo = absCosTheta(wo);
    float nDotWi = absCosTheta(wi);
    float woDotWh = std::abs(dot(wo, wh));
    return std::min(
        1.0f, 
        std::min(
            2.0f * nDotWh * nDotWo /woDotWh,
            2.0f * nDotWh * nDotWi / woDotWh
            )
        );
}

