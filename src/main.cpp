#include "util/tgaImage.h"
#define _USE_MATH_DEFINES
#include <math.h>
#undef _USE_MATH_DEFINES

const int kWidth = 800;
const int kHeight = 600;
const float kFov = M_PI / 2.0f;
const std::string kFrameBufferOutput = "./output.tga";
const float kMaxDistance = 1000.f;
const size_t kMaxReflectionTimes = 4;
const float kReflectionOffset = 1e-3f;
const float kCheckerboardThreshold = 1e-3f;
const float kCheckerboardPlaneY = -4.f;
const float kCheckerboardPlaneXMin = -10.f;
const float kCheckerboardPlaneXMax = 10.f;
const float kCheckerboardPlaneZMin = -30.f;
const float kCheckerboardPlaneZMax = -10.f;

struct Light {
    Light(const Vec3f& pos, const float& i) : position_(pos), intensity_(i){};

    Vec3f position_;
    float intensity_;
};

struct Material {
    Material() = default;
    Material(const float& r, const Vec4f& a, const Vec3f& color, const float& spec)
        : refractiveIndex_(r), albedo_(a), diffuse_(color), specularExponent_(spec) {}
    float refractiveIndex_{1.f};
    Vec4f albedo_{1.f, 0.f, 0.f, 0.f};
    Vec3f diffuse_;
    float specularExponent_{0.f};
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

bool scene_intersect(const Vec3f& ori, const Vec3f& dir, const std::vector<Sphere>& spheres, Vec3f& hit,
                     Vec3f& hitNormal, Material& material) {
    float sphereDistance = std::numeric_limits<float>::max();
    for (const auto& sphere : spheres) {
        float distance;
        if (sphere.ray_intersect(ori, dir, distance) && distance < sphereDistance) {
            sphereDistance = distance;
            hit = ori + dir * distance;
            hitNormal = (hit - sphere.center_).normalize();
            material = sphere.material_;
        }
    }
    float checkerboardDistance = std::numeric_limits<float>::max();
    if (std::fabs(dir.y) > kCheckerboardThreshold) {
        float d = (kCheckerboardPlaneY - ori.y) / dir.y;
        Vec3f point = ori + dir * d;
        if (d > 0 && point.x > kCheckerboardPlaneXMin && point.x < kCheckerboardPlaneXMax &&
            point.z > kCheckerboardPlaneZMin && point.z < kCheckerboardPlaneZMax && d < sphereDistance) {
            checkerboardDistance = d;
            hit = point;
            hitNormal = {0.f, 1.f, 0.f};
            material.diffuse_ = ((static_cast<int>(.5 * hit.x + 1000) + static_cast<int>(.5 * hit.z)) & 1)
                                    ? Vec3f{.3f, .3f, .3f}
                                    : Vec3f{.3f, .2f, .1f};
        }
    }
    return std::min(sphereDistance, checkerboardDistance) < kMaxDistance;
}

Vec3f reflect(const Vec3f& in, const Vec3f& normal) { return in - normal * 2.f * (in * normal); }

// Snell's law
Vec3f refract(const Vec3f& in, const Vec3f& normal, const float& refractiveIndex) {
    float cosIn = -std::max(-1.f, std::min(1.f, in * normal));
    float etaIn = 1.f, etaT = refractiveIndex;
    Vec3f n = normal;

    // if the ray is inside the object, swap the indices and invert the normal to get the correct result
    if (cosIn < 0) {
        cosIn = -cosIn;
        std::swap(etaIn, etaT);
        n = -normal;
    }
    float eta = etaIn / etaT;
    float k = 1 - eta * eta * (1 - cosIn * cosIn);
    return k < 0 ? Vec3f(0.f, 0.f, 0.f) : in * eta + n * (eta * cosIn - sqrtf(k));
}

Vec3f cast_ray(const Vec3f& org, const Vec3f& dir, const std::vector<Sphere>& spheres, const std::vector<Light>& lights,
               size_t depth = 0) {
    Vec3f background{0.2, 0.7, 0.8};
    Vec3f hitPoint, hitNormal;
    Material material;

    if (depth > kMaxReflectionTimes || !scene_intersect(org, dir, spheres, hitPoint, hitNormal, material)) {
        return background;
    }

    const Vec3f reflectDir = reflect(dir, hitNormal).normalize();
    const Vec3f refractDir = refract(dir, hitNormal, material.refractiveIndex_).normalize();
    const Vec3f reflectOri = reflectDir * hitNormal < 0 ? hitPoint - hitNormal * kReflectionOffset
                                                        : hitPoint + hitNormal * kReflectionOffset;
    const Vec3f refractOri = refractDir * hitNormal < 0 ? hitPoint - hitNormal * kReflectionOffset
                                                        : hitPoint + hitNormal * kReflectionOffset;
    const Vec3f refectColor = cast_ray(reflectOri, reflectDir, spheres, lights, depth + 1);
    const Vec3f refractColor = cast_ray(refractOri, refractDir, spheres, lights, depth + 1);

    float diffuseLightIntensity = 0;
    float specularLightIntensity = 0;
    for (const auto& light : lights) {
        const Vec3f lightDir = (light.position_ - hitPoint).normalize();
        const float lightDistance = (light.position_ - hitPoint).norm();

        const Vec3f shadowOri = lightDir * hitNormal < 0 ? hitPoint - hitNormal * kReflectionOffset
                                                         : hitPoint + hitNormal * kReflectionOffset;
        Vec3f shadowPoint, shadowNormal;
        Material shadowMaterial;
        if (scene_intersect(shadowOri, lightDir, spheres, shadowPoint, shadowNormal, shadowMaterial) &&
            (shadowPoint - shadowOri).norm() < lightDistance)
            continue;
        diffuseLightIntensity += light.intensity_ * std::max(0.f, lightDir * hitNormal);
        // -reflect(-lightDir, hitNormal) = reflect(lightDir, hitNormal)
        specularLightIntensity +=
            powf(std::max(0.f, reflect(lightDir, hitNormal) * dir), material.specularExponent_) * light.intensity_;
    }
    Vec3f out = material.diffuse_ * diffuseLightIntensity * material.albedo_[0] +
                Vec3f(1.f, 1.f, 1.f) * specularLightIntensity * material.albedo_[1] +
                refectColor * material.albedo_[2] + refractColor * material.albedo_[3];
    float max = std::max(out[0], std::max(out[1], out[2]));
    if (max > 1.f) out = out * (1 / max);
    return out;
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

    Material ivory(1.0f, {0.6f, 0.3f, 0.1f, 0.0f}, {0.4f, 0.4f, 0.3f}, 50.f);
    Material glass(1.5, {0.0f, 0.5f, 0.1f, 0.8f}, {0.6f, 0.7f, 0.8f}, 125.f);
    Material redRubber(1.0, {0.9f, 0.1f, 0.0f, 0.0f}, {0.3f, 0.1f, 0.1f}, 10.f);
    Material mirror(1.0, {0.0f, 10.f, 0.8f, 0.0f}, {1.0f, 1.0f, 1.0f}, 1425.f);

    std::vector<Sphere> spheres{
        {Vec3f{-3.f, 0.f, -16.f}, 2, ivory},
        {Vec3f{-1.f, -1.5f, -12.f}, 2, glass},
        {Vec3f{1.5f, -0.5f, -18.f}, 3, redRubber},
        {Vec3f{7.f, 5.f, -18.f}, 4, mirror},
    };

    std::vector<Light> lights{{{-20.f, 20.f, 20.f}, 1.5}, {{30.f, 50.f, -25.f}, 1.8f}, {{30.f, 20.f, 30.f}, 1.7f}};

    render(frameBuffer, spheres, lights);
    frameBuffer.write_tga_file(kFrameBufferOutput.c_str());
    return 0;
}