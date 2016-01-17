#if !defined(LIGHT_H)
#define LIGHT_H

#include <memory>

#include "constants.h"
#include "shape.h"
#include "spectrum.h"
#include "vector.h"
#include "qmc.h"

class Light {
public:
	virtual ~Light() = 0;

	virtual Spectrum sample(const Vector3f& scenePosition, Vector3f* wi,
		float* pdf, Vector3f* sampledPosition, float u1, float u2) const = 0;

	virtual Spectrum power() const = 0;

    virtual Spectrum intensity() const = 0;

	virtual bool isDelta() const = 0;
};

inline Light::~Light()
{
}

class PointLight : public Light {
public:
	PointLight(Vector3f position, Spectrum intensity)
		: position_(position)
		, intensity_(intensity)
	{ }

	~PointLight() { }

	Spectrum sample(const Vector3f& scenePosition, Vector3f* wi, float* pdf,
		Vector3f* sampledPosition, float u1, float u2) const override
	{
		((void)u1);
		((void)u2);
		Vector3f toLight = position_ - scenePosition;
		*wi = normal(toLight);
		*pdf = 1.0f;
		*sampledPosition = position_;
		return intensity_ / length2(toLight); 
	}

	Spectrum power() const override
	{
		return intensity_ * 4.0f * PI;
	}

    Spectrum intensity() const override
    {
        return intensity_;
    }

	bool isDelta() const override
	{
		return true;
	}

private:
	Vector3f position_;
	Spectrum intensity_;
};

class AreaLight : public Light {
public:
	AreaLight(std::shared_ptr<Shape> emitter, Spectrum intensity)
		: emitter_(emitter)
		, intensity_(intensity)
	{
		area_ = emitter->area();
	}

	Spectrum sample(const Vector3f& scenePosition, Vector3f* wi, float* pdf,
		Vector3f* sampledPosition, float u1, float u2) const override
	{
		Vector3f pos = emitter_->sample(u1, u2, pdf);
		Vector3f toLight = pos - scenePosition;
		*wi = normal(toLight);
		*sampledPosition = pos;
		return intensity_ / length2(toLight);
	}

	// Look into this
	Spectrum power() const override
	{
		return intensity_ * area_ * PI;
	}

    Spectrum intensity() const override
    {
        return intensity_;
    }

	bool isDelta() const override
	{
		return false;
	}

private:
	std::shared_ptr<Shape> emitter_;
	Spectrum intensity_;
	float area_;
};

#endif // LIGHT_H

