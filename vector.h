#if !defined(VECTOR_H)
#define VECTOR_H

#include <cmath>
#include <cstdint>
#include <type_traits>

template <typename T>
struct TVector3 {
	static_assert(std::is_arithmetic<T>::value, "Only numbers allowed");
	T x;
	T y;
	T z;

	explicit TVector3() = default;

	explicit TVector3(T val) : x(val), y(val), z(val) { }

	explicit TVector3(T x, T y, T z) : x(x), y(y), z(z) { }

	TVector3(const TVector3& copy) = default;

	TVector3& operator=(const TVector3& copy) = default;

	TVector3(TVector3&& move) = default;

	TVector3& operator=(TVector3&& move) = default;

	inline TVector3 operator+(const TVector3& rhs) const
	{
		return TVector3(x + rhs.x, y + rhs.y, z + rhs.z);
	}

	inline TVector3 operator-(const TVector3& rhs) const
	{
		return TVector3(x - rhs.x, y - rhs.y, z - rhs.z);
	}
	
	/*
	 * THIS IS TEMPORARY!!!
	 * When spectrum class is implemented, delete this
	 */
	[[deprecated]]
	inline TVector3 operator/(const TVector3& rhs)
	{
		return TVector3(x / rhs.x, y / rhs.y, z / rhs.z);
	}

	inline TVector3 operator+(const T& rhs) const
	{
		return TVector3(x + rhs, y + rhs, z + rhs);
	}

	inline TVector3 operator-(const T& rhs) const
	{
		return TVector3(x - rhs, y - rhs, z - rhs);
	}

	inline TVector3 operator*(const T& rhs) const
	{
		return TVector3(x * rhs, y * rhs, z * rhs);
	}

	inline TVector3 operator/(const T& rhs) const
	{
		assert(rhs != 0.0f);
		return TVector3(x / rhs, y / rhs, z / rhs);
	}

	inline T length() const
	{
		return std::sqrt(x * x + y * y + z * z);
	}

	inline T length2() const
	{
		return x * x + y * y + z * z;
	}

	inline TVector3 pointwise(const TVector3& rhs) const
	{
		return TVector3(x * rhs.x, y * rhs.y, z * rhs.z);
	}

	inline T dot(const TVector3& rhs) const
	{
		return x * rhs.x + y * rhs.y + z * rhs.z;
	}

	inline TVector3 cross(const TVector3& rhs) const
	{
		return TVector3(
			y * rhs.z - z * rhs.y,
			z * rhs.x - x * rhs.z,
			x * rhs.y - y * rhs.x
		);
	}

	inline TVector3 normal() const
	{
		return *this * (1.0f / length());
	}

	inline TVector3 operator-() const
	{
		return TVector3(-x, -y, -z);
	}
};

template <typename T>
inline TVector3<T> operator*(T lhs, const TVector3<T>& rhs)
{
	return rhs * lhs;
}

template <typename T>
inline T length(const TVector3<T>& v)
{
	return v.length();
}

template <typename T>
inline T length2(const TVector3<T>& v)
{
	return v.length2();
}

template <typename T>
inline TVector3<T> pointwise(const TVector3<T>& lhs, const TVector3<T>& rhs)
{
	return lhs.pointwise(rhs);
}

template <typename T>
inline T dot(const TVector3<T>& lhs, const TVector3<T>& rhs)
{
	return lhs.dot(rhs);
}

template <typename T>
inline TVector3<T> cross(const TVector3<T> lhs, const TVector3<T>& rhs)
{
	return lhs.cross(rhs);
}

template <typename T>
inline TVector3<T> normal(const TVector3<T>& v)
{
	return v.normal();
}

using Vector3f = TVector3<float>;
using Vector3i = TVector3<int32_t>;

using Spectrum = TVector3<float>;

#endif // VECTOR_H
