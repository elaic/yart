#if !defined(RENDERER_H)
#define RENDERER_H

#include <cstdint>

class Scene;
class Camera;

class Renderer {
public:
    void render(const Scene& scene, Camera& camera) const;

private:
    static constexpr int32_t tileSize_ = 32;
};

#endif // RENDERER_H

