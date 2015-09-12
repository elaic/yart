#if !defined(CAMERA_H)
#define CAMERA_H

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
    {
        right_ = Vector3f(width_ * fov_ / height_, 0.0f, 0.0f);
        up_ = normal(cross(right_, direction_)) * fov_;
    }

    Ray sample(float x, float y) const
    {
        Vector3f d =
            right_ * ((x + 0.5f) / width_ - 0.5f) +
            up_ * (-(y + 0.5f) / height_ + 0.5f) + direction_;

        return Ray(position_ + d * 140.0f, normal(d));
    }

private:
    Vector3f position_;
    Vector3f direction_;
    int32_t width_;
    int32_t height_;
    float fov_;
    Vector3f up_;
    Vector3f right_;
};

#endif // CAMERA_H

