#if !defined(LIGHT_H)
#define LIGHT_H

#include "constants.h"
#include "vector.h"

class PointLight {
public:
	PointLight(Vector3f position, Spectrum intensity)
	: position_(position)
	, intensity_(intensity)
	{ }

	Spectrum sample(const Vector3f& scenePosition, Vector3f* wi, float* pdf)
	{
		Vector3f toLight = position_ - scenePosition;
		*wi = normal(toLight);
		*pdf = 1.0f;
		return intensity_ * (1.0f / length2(toLight)); 
	}

	Spectrum power() const
	{
		return intensity_ * 4.0f * PI;
	}

	bool isDelta() const
	{
		return true;
	}

	const Vector3f& position() const
	{
	 	return position_;
	}
private:
	Vector3f position_;
	Spectrum intensity_;
};

#endif // LIGHT_H
