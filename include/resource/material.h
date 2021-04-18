#pragma once
#include "util/geometry.h"
#include "util/tgaImage.h"

struct RayTracingMaterial {
    RayTracingMaterial() = default;
    RayTracingMaterial(const float& r, const Vec4f& a, const Vec3f& color, const float& spec)
        : refractiveIndex_(r), albedo_(a), diffuse_(color), specularExponent_(spec) {}
    float refractiveIndex_{1.f};
    Vec4f albedo_{1.f, 0.f, 0.f, 0.f};
    Vec3f diffuse_;
    float specularExponent_{0.f};
};

class Material {
   public:
    Material(const std::string& diffuseFile, const std::string& normalFile = "", const std::string& specularFile = "");
    ~Material() = default;

    TGAColor diffuse(Vec2f uv);
    Vec3f normal(Vec2f uv);
    float specular(Vec2f uv);

   private:
    TGAImage diffuseMap_;
    TGAImage normalMap_;
    TGAImage specularMap_;
};