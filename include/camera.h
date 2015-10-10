#if !defined(CAMERA_H)
#define CAMERA_H

#include "bitmap.h"
#include "vector.h"

class Camera {
public:
    Camera(const Vector3f& position, const Vector3f& direction,
        int32_t width, int32_t height, float fov,
        const Vector3f up = Vector3f(0.0f, 1.0f, 0.0f))
        : position_(position)
        , direction_(direction)
        , width_(width)
        , height_(height)
        , fov_(fov)
        , up_(up)
        , sensor_(Bitmap(width_, height_))
    {
        right_ = Vector3f(width_ * fov_ / height_, 0.0f, 0.0f);
        up_ = normal(cross(right_, direction_)) * fov_;
    }

    Ray sample(float x, float y) const
    {
        Vector3f d =
            right_ * ((x + 0.5f) / width_ - 0.5f) +
            up_ * (-(y + 0.5f) / height_ + 0.5f) + direction_;

        return Ray(position_, normal(d));
    }

	inline int32_t getWidth() const
	{
		return width_;
	}

	inline int32_t getHeight() const
	{
		return height_;
	}

	inline Vector2i getResolution() const
	{
		return Vector2i(width_, height_);
	}

    void accumulate(int32_t x, int32_t y, const RGBColor& color)
    {
        sensor_.set(x, y, color);
    }

    void saveImage(const std::string& name)
    {
        sensor_.write(name);
    }

private:
    Vector3f position_;
    Vector3f direction_;
    int32_t width_;
    int32_t height_;
    float fov_;
    Vector3f up_;
    Vector3f right_;

    // TODO: remove bitmap class (it doesn't make any sense), implement Sensor
    // class, in it develop method which will convert the data to bitmap data
    // and just leave the save bitmap function
    Bitmap sensor_;
};

#endif // CAMERA_H

