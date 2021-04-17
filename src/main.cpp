#include "util/tgaImage.h"
#define _USE_MATH_DEFINES
#include <math.h>
#undef _USE_MATH_DEFINES

const int kWidth = 800;
const int kHeight = 600;
const float kFov = M_PI / 2.0f;
const std::string kFrameBufferOutput = "./output.tga";

struct Light {
    Light(const Vec3f& pos, const float& i) : position_(pos), intensity_(i){};

    Vec3f position_;
    float intensity_;
};

struct Material {
    Material() = default;
    Material(const TGAColor& color) : diffuse_(color){};
    Material(const Vec3f& color) {
        diffuse_ = TGAColor(static_cast<unsigned char>(color.x * 255), static_cast<unsigned char>(color.y * 255),
                            static_cast<unsigned char>(color.z * 255), 255);
    }
    Material(const Vec4f& color) {
        diffuse_ = TGAColor(static_cast<unsigned char>(color[0] * 255), static_cast<unsigned char>(color[1] * 255),
                            static_cast<unsigned char>(color[2] * 255), static_cast<unsigned char>(color[3] * 255));
    }

    TGAColor diffuse_;
};

struct Sphere {
    Vec3f center_;
    float radius_;
    Material material_;

    Sphere(const Vec3f& c, const float& r) : center_(c), radius_(r){};
    Sphere(const Vec3f& c, const float& r, const Material& m) : center_(c), radius_(r), material_(m){};

    // dir is normalized
    bool ray_intersect(const Vec3f& org, const Vec3f& dir, float& t0) const {
        Vec3f l = center_ - org;
        float tca = l * dir;
        float d2 = l * l - tca * tca;
        if (d2 > radius_ * radius_) return false;

        float thc = sqrtf(radius_ * radius_ - d2);
        t0 = tca - thc;
        float t1 = tca + thc;
        if (t0 < 0) t0 = t1;
        if (t0 < 0) return false;
        return true;
    }
};

bool scene_intersect(const Vec3f& org, const Vec3f& dir, const std::vector<Sphere>& spheres, Vec3f& hit,
                     Vec3f& hitNormal, Material& material) {
    float sphereDistance = std::numeric_limits<float>::max();
    for (const auto& sphere : spheres) {
        float distance;
        if (sphere.ray_intersect(org, dir, distance) && distance < sphereDistance) {
            sphereDistance = distance;
            hit = org + dir * distance;
            hitNormal = (hit - sphere.center_).normalize();
            material = sphere.material_;
        }
    }
    return sphereDistance < 1000.f;
}

TGAColor cast_ray(const Vec3f& org, const Vec3f& dir, const std::vector<Sphere>& spheres, const std::vector<Light>& lights) {
    TGAColor background{51, 179, 204, 255};
    Vec3f hitPoint, hitNormal;
    Material material;

    if (!scene_intersect(org, dir, spheres, hitPoint, hitNormal, material)) {
        return background;
    }
    float diffuseLightIntensity = 0;
    for (const auto& light : lights) {
        const Vec3f lightDir = (light.position_ - hitPoint).normalize();
        diffuseLightIntensity += light.intensity_ * std::max(0.f, lightDir * hitNormal);
    }
    return material.diffuse_ * diffuseLightIntensity;
}

void render(TGAImage& frameBuffer, const std::vector<Sphere>& spheres, const std::vector<Light>& lights) {
    const Vec3f cameraPos{0.f, 0.f, 0.f};
    for (size_t i = 0; i < kWidth; ++i)
        for (size_t j = 0; j < kHeight; ++j) {
            const float x = (2 * (i + 0.5) / static_cast<float>(kWidth) - 1) * tan(kFov / 2.f) * kWidth /
                            static_cast<float>(kHeight);
            const float y = -(2 * (j + 0.5) / static_cast<float>(kHeight) - 1) * tan(kFov / 2.f);
            Vec3f dir = Vec3f(x, y, -1).normalize();
            frameBuffer.set(i, j, cast_ray(cameraPos, dir, spheres, lights));
        }
}

int main() {
    TGAImage frameBuffer(kWidth, kHeight, TGAImage::RGB);

    Material ivory({0.4f, 0.4f, 0.3f});
    Material redRubber({0.3f, 0.1f, 0.1f});

    std::vector<Sphere> spheres{
        {Vec3f{-3.f, 0.f, -16.f}, 2, ivory},
        {Vec3f{-1.f, -1.5f, -12.f}, 2, redRubber},
        {Vec3f{1.5f, -0.5f, -18.f}, 3, redRubber},
        {Vec3f{7.f, 5.f, -18.f}, 4, ivory},
    };

    std::vector<Light> lights {
        { {-20.f, 20.f, 20.f}, 1.5 }
    };

    render(frameBuffer, spheres, lights);
    frameBuffer.write_tga_file(kFrameBufferOutput.c_str());
    return 0;
}