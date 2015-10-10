#if !defined(QMC_H)
#define QMC_H

#include "vector.h"

inline void concentricSampleDisk(float u1, float u2, float* dx, float* dy)
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

#endif // QMC_H

