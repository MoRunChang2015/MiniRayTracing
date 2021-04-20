#include <iostream>

#include "util/tgaImage.h"
#define _USE_MATH_DEFINES
#include <math.h>
#undef _USE_MATH_DEFINES

#include "concurrent/concurrentModule.h"
#include "resource/cubemap.h"
#include "resource/material.h"
#include "resource/model.h"
#include "resource/sphere.h"

const int kWidth = 800;
const int kHeight = 600;
const float kFov = M_PI / 3.0f;
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
const float kFloatLimit = 1e-5f;
const size_t kWorkersNum = 4;
const std::string kDuckObj = "../res/duck.obj";
const std::string kDiabloObj = "../res/diablo3_pose/diablo3_pose.obj";
const std::string kDiabloDiffuse = "../res/diablo3_pose/diablo3_pose_diffuse.tga";
const std::string kDiabloNormal = "../res/diablo3_pose/diablo3_pose_nm_tangent.tga";
const std::string kDiabloSpec = "../res/diablo3_pose/diablo3_pose_spec.tga";

const CubeMap skyBox{{
    "../res/skybox/sky10ft.tga",
    "../res/skybox/sky10up.tga",
    "../res/skybox/sky10rt.tga",
    "../res/skybox/sky10bk.tga",
    "../res/skybox/sky10dn.tga",
    "../res/skybox/sky10lf.tga",
}};

struct Light {
    Light(const Vec3f& pos, const float& i) : position_(pos), intensity_(i){};

    Vec3f position_;
    float intensity_;
};

bool scene_intersect(const Vec3f& ori, const Vec3f& dir, const std::vector<Sphere>& spheres,
                     const std::vector<Model>& models, Vec3f& hit, Vec3f& hitNormal, RayTracingMaterial& material) {
    float minDistance = std::numeric_limits<float>::max();
    for (const auto& sphere : spheres) {
        float distance;
        if (sphere.ray_intersect(ori, dir, distance) && distance < minDistance) {
            minDistance = distance;
            hit = ori + dir * distance;
            hitNormal = (hit - sphere.center_).normalize();
            material = sphere.material_;
        }
    }
    for (const auto& model : models) {
        float distance;
        float u, v;
        Vec3f hitNormal_;
        if (model.ray_intersect(ori, dir, distance, u, v, hitNormal_) && distance < minDistance) {
            minDistance = distance;
            hit = ori + dir * distance;
            hitNormal = hitNormal_;
            material = model.get_ray_tracing_material(u, v);
        }
    }

    float checkerboardDistance = std::numeric_limits<float>::max();
    if (std::fabs(dir.y) > kCheckerboardThreshold) {
        float d = (kCheckerboardPlaneY - ori.y) / dir.y;
        Vec3f point = ori + dir * d;
        if (d > 0 && point.x > kCheckerboardPlaneXMin && point.x < kCheckerboardPlaneXMax &&
            point.z > kCheckerboardPlaneZMin && point.z < kCheckerboardPlaneZMax && d < minDistance) {
            checkerboardDistance = d;
            hit = point;
            hitNormal = {0.f, 1.f, 0.f};
            material.albedo_ = {1.0f, 0.f, 0.f, 0.f};
            material.diffuse_ = ((static_cast<int>(.5 * hit.x + 1000) + static_cast<int>(.5 * hit.z)) & 1)
                                    ? Vec3f{.3f, .3f, .3f}
                                    : Vec3f{.3f, .2f, .1f};
            material.specularExponent_ = 10.f;
        }
    }
    return std::min(minDistance, checkerboardDistance) < kMaxDistance;
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

Vec3f cast_ray(const Vec3f& ori, const Vec3f& dir, const std::vector<Sphere>& spheres, const std::vector<Model>& models,
               const std::vector<Light>& lights, size_t depth = 0) {
    Vec3f hitPoint, hitNormal;
    RayTracingMaterial material;

    if (depth > kMaxReflectionTimes || !scene_intersect(ori, dir, spheres, models, hitPoint, hitNormal, material)) {
        return skyBox.sample(dir).to_linear();
    }

    Vec3f reflectColor{0.f, 0.f, 0.f}, refractColor{0.f, 0.f, 0.f};
    if (material.albedo_[2] > kFloatLimit) {
        const Vec3f reflectDir = reflect(dir, hitNormal).normalize();
        const Vec3f reflectOri = reflectDir * hitNormal < 0 ? hitPoint - hitNormal * kReflectionOffset
                                                            : hitPoint + hitNormal * kReflectionOffset;
        reflectColor = cast_ray(reflectOri, reflectDir, spheres, models, lights, depth + 1);
    }

    if (material.albedo_[3] > kFloatLimit) {
        const Vec3f refractDir = refract(dir, hitNormal, material.refractiveIndex_).normalize();

        const Vec3f refractOri = refractDir * hitNormal < 0 ? hitPoint - hitNormal * kReflectionOffset
                                                            : hitPoint + hitNormal * kReflectionOffset;
        refractColor = cast_ray(refractOri, refractDir, spheres, models, lights, depth + 1);
    }

    float diffuseLightIntensity = 0;
    float specularLightIntensity = 0;
    for (const auto& light : lights) {
        const Vec3f lightDir = (light.position_ - hitPoint).normalize();
        const float lightDistance = (light.position_ - hitPoint).norm();

        const Vec3f shadowOri = lightDir * hitNormal < 0 ? hitPoint - hitNormal * kReflectionOffset
                                                         : hitPoint + hitNormal * kReflectionOffset;
        Vec3f shadowPoint, shadowNormal;
        RayTracingMaterial shadowMaterial;
        if (scene_intersect(shadowOri, lightDir, spheres, models, shadowPoint, shadowNormal, shadowMaterial) &&
            (shadowPoint - shadowOri).norm() < lightDistance)
            continue;
        diffuseLightIntensity += light.intensity_ * std::max(0.f, lightDir * hitNormal);
        // -reflect(-lightDir, hitNormal) = reflect(lightDir, hitNormal)
        specularLightIntensity +=
            powf(std::max(0.f, reflect(lightDir, hitNormal) * dir), material.specularExponent_) * light.intensity_;
    }
    Vec3f out = material.diffuse_ * diffuseLightIntensity * material.albedo_[0] +
                Vec3f(1.f, 1.f, 1.f) * specularLightIntensity * material.albedo_[1] +
                reflectColor * material.albedo_[2] + refractColor * material.albedo_[3];
    float max = std::max(out[0], std::max(out[1], out[2]));
    if (max > 1.f) out = out * (1 / max);
    return out;
}

void render(TGAImage& frameBuffer, const std::vector<Sphere>& spheres, const std::vector<Model>& models,
            const std::vector<Light>& lights) {
    const Vec3f cameraPos{0.f, 0.f, 0.f};

    ConcurrentModule::parallelFor(
        kWidth * kHeight, [&cameraPos, &spheres, &models, &lights, &frameBuffer](const auto& index) {
            const int i = index / kHeight;
            const int j = index % kHeight;
            const float x = (2 * (i + 0.5) / static_cast<float>(kWidth) - 1) * tan(kFov / 2.f) * kWidth /
                            static_cast<float>(kHeight);
            const float y = -(2 * (j + 0.5) / static_cast<float>(kHeight) - 1) * tan(kFov / 2.f);
            const Vec3f dir = Vec3f(x, y, -1).normalize();
            frameBuffer.set(i, j, cast_ray(cameraPos, dir, spheres, models, lights));
            std::cout << "Ray tracing (" << i << " , " << j << ")..Done" << std::endl;
        });
}

int main() {
    TGAImage frameBuffer(kWidth, kHeight, TGAImage::RGB);

    RayTracingMaterial ivory(1.0f, {0.6f, 0.3f, 0.1f, 0.0f}, {0.4f, 0.4f, 0.3f}, 50.f);
    RayTracingMaterial glass(1.5, {0.0f, 0.5f, 0.1f, 0.8f}, {0.6f, 0.7f, 0.8f}, 125.f);
    RayTracingMaterial redRubber(1.0f, {0.9f, 0.1f, 0.0f, 0.0f}, {0.3f, 0.1f, 0.1f}, 10.f);
    RayTracingMaterial mirror(1.0f, {0.0f, 10.f, 0.8f, 0.0f}, {1.0f, 1.0f, 1.0f}, 1425.f);

    std::vector<Sphere> spheres{
        {Vec3f{-3.f, 0.f, -18.f}, 2, ivory},
        {Vec3f{-2.f, -1.5f, -8.f}, 2, glass},
        {Vec3f{1.5f, -0.5f, -20.f}, 3, redRubber},
        {Vec3f{7.f, 5.f, -20.f}, 4, mirror},
    };

    Mesh duckMesh(kDuckObj);
    Mesh diabloMesh(kDiabloObj);

    Material diabloMat{kDiabloDiffuse, kDiabloNormal, kDiabloSpec};

    std::vector<Model> models{{&duckMesh, nullptr, glass},
                              {&diabloMesh, &diabloMat, {1.0f, {1.0f, 0.0f, 0.0f, 0.0f}, {0.f, 0.f, 0.f}, 0.f}}};

    Matrix4x4 duckTransform = Matrix4x4::identity();
    duckTransform[3] = {1.f, 0.f, -4.f, 1.f};

    Matrix4x4 diabloTransform = Matrix4x4::identity();
    diabloTransform[0][0] = diabloTransform[1][1] = diabloTransform[2][2] = 4.f;
    diabloTransform[3] = {0.f, 0.f, -14.f, 1.f};
    models[0].set_transform(duckTransform);
    models[1].set_transform(diabloTransform);

    std::vector<Light> lights{{{-20.f, 20.f, 20.f}, 1.5}, {{30.f, 50.f, -25.f}, 1.8f}, {{30.f, 20.f, 30.f}, 1.7f}};

    ConcurrentModule::getInstance().setNumWorkerThreads(kWorkersNum);

    render(frameBuffer, spheres, models, lights);

    frameBuffer.write_tga_file(kFrameBufferOutput.c_str());
    return 0;
}