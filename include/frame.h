#if !defined(FRAME_H)
#define FRAME_H

#include "vector.h"

class Frame {
public:
	Frame()
	: s(Vector3f(1.0f, 0.0f, 0.0f))
	, t(Vector3f(0.0f, 1.0f, 0.0f))
	, n(Vector3f(0.0f, 0.0f, 1.0f))
	{ }

	Frame(const Vector3f& x, const Vector3f& y, const Vector3f z)
	: s(x), t(y), n(z)
	{ }

	Frame(const Vector3f& normal)
	{
		using std::abs;
		using std::sqrt;
		n = ::normal(normal);	
		if (abs(n.x) > abs(n.y)) {
			float invLen = 1.0f / sqrt(n.x * n.x + n.z * n.z);
			t = Vector3f(-n.z * invLen, 0.0f, n.x * invLen);
		} else {
			float invLen = 1.0f / sqrt(n.y * n.y + n.z * n.z);
			t = Vector3f(0.0f, n.z * invLen, -n.y * invLen);
		}
		s = cross(n, t);
	}

	inline Vector3f toLocal(const Vector3f& world) const
	{
		return Vector3f(
			dot(world, s),
			dot(world, t),
			dot(world, n)
		);
	}

	inline Vector3f toWorld(const Vector3f& local) const
	{
		return local.x * s + local.y * t + local.z * n;
	}

public:
	Vector3f s;
	Vector3f t;
	Vector3f n;
};

#endif // FRAME_H

