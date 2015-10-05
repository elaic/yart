#if !defined(SPECTRUM_H)
#define SPECTRUM_H

#include <cassert>

struct RGBColor
{
	float r;
	float g;
	float b;
};

class Spectrum {
public:
	Spectrum() { }

	Spectrum(float r, float g, float b)
	{
		samples[0] = r;
		samples[1] = g;
		samples[2] = b;
	}

	// Think about this being explicit...
	Spectrum(float val)
	{
		samples[0] = samples[1] = samples[2] = val;
	}

	Spectrum(const Spectrum& copy) = default;

	Spectrum(Spectrum&& move) = default;

	Spectrum& operator=(const Spectrum& copy) = default;

	Spectrum& operator=(Spectrum&& move) = default;

	/*
	 * Get luma from spectrum
	 */
	float y() const
	{
		return 0.2126f * samples[0] + 0.7152f * samples[1] + 0.0722f * samples[2];
	}

	bool isBlack() const
	{
		for (auto i = 0; i < 3; ++i)
			if (samples[i] != 0.0f)
				return false;

		return true;
	}

	RGBColor toRGB() const
	{
		return RGBColor{ samples[0], samples[1], samples[2] };
	}

	Spectrum& operator+=(const Spectrum& rhs)
	{
		for (auto i = 0; i < 3; ++i)
			samples[i] += rhs.samples[i];

		return *this;
	}

	Spectrum& operator-=(const Spectrum& rhs)
	{
		for (auto i = 0; i < 3; ++i)
			samples[i] -= rhs.samples[i];

		return *this;
	}

	Spectrum& operator*=(float rhs)
	{
		for (auto i = 0; i < 3; ++i)
			samples[i] *= rhs;

		return *this;
	}

	Spectrum& operator*=(const Spectrum& rhs)
	{
		for (auto i = 0; i < 3; ++i)
			samples[i] *= rhs.samples[i];

		return *this;
	}

	Spectrum& operator/=(float rhs)
	{
		assert(rhs != 0);
		float recip = 1.0f / rhs;
		for (auto i = 0; i < 3; ++i)
			samples[i] *= recip;

		return *this;
	}

	Spectrum& operator/=(const Spectrum& rhs)
	{
		for (auto i = 0; i < 3; ++i)
			samples[i] /= rhs.samples[i];

		return *this;
	}

	Spectrum operator+(const Spectrum& rhs)
	{
		Spectrum result;

		for (auto i = 0; i < 3; ++i)
			result.samples[i] = samples[i] + rhs.samples[i];

		return result;
	}

	Spectrum operator-(const Spectrum& rhs)
	{
		Spectrum result;

		for (auto i = 0; i < 3; ++i)
			result.samples[i] = samples[i] - rhs.samples[i];

		return result;
	}

	Spectrum operator*(float rhs) const
	{
		Spectrum result;

		for (auto i = 0; i < 3; ++i)
			result.samples[i] = samples[i] * rhs;

		return result;
	}

	friend inline Spectrum operator*(float lhs, const Spectrum& rhs)
	{
		return rhs * lhs;
	}

	Spectrum operator*(const Spectrum& rhs) const
	{
		Spectrum result;

		for (auto i = 0; i < 3; ++i)
			result.samples[i] = samples[i] * rhs.samples[i];

		return result;
	}

	Spectrum operator/(float rhs) const
	{
		assert(rhs != 0);
		Spectrum result;
		float recip = 1.0f / rhs;

		for (auto i = 0; i < 3; ++i)
			result.samples[i] = samples[i] * recip;

		return result;
	}

	Spectrum operator/(const Spectrum& rhs) const
	{
		Spectrum result;

		for (auto i = 0; i < 3; ++i) {
			assert(rhs.samples[i] != 0);
			result.samples[i] = samples[i] / rhs.samples[i];
		}

		return result;
	}

	bool operator==(const Spectrum& rhs) const
	{
		for (auto i = 0; i < 3; ++i)
			if (samples[i] != rhs.samples[i])
				return false;

		return true;
	}

	bool operator!=(const Spectrum& rhs) const
	{
		for (auto i = 0; i < 3; ++i)
			if (samples[i] == rhs.samples[i])
				return false;

		return true;
	}

private:
	float samples[3];
};

#endif // SPECTRUM_H
