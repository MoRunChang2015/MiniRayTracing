#pragma once

#include <vector>

#include "resource/material.h"
#include "resource/mesh.h"
#include "util/geometry.h"

class Model {
   public:
    Model(Mesh* mesh, Material* material, const RayTracingMaterial& rm)
        : meshRes_(mesh),
          materialRes_(material),
          rayTracingMaterial_(rm),
          transform_(Matrix4x4::identity()),
          invertTransform_(Matrix4x4::identity()){};

    Mesh* get_mesh() const { return meshRes_; }
    Material* get_material() const { return materialRes_; }

    const Matrix4x4& get_transform() const { return transform_; }
    void set_transform(const Matrix4x4& m) {
        transform_ = m;
        invertTransform_ = transform_.invert();
    }

    bool ray_intersect(const Vec3f& ori, const Vec3f& dir, float& t0, float& u, float& v, Vec3f& hitNormal) const;

    RayTracingMaterial get_raytracing_material(const float& u, const float& v) const;

   private:
    Mesh* meshRes_{nullptr};
    Material* materialRes_{nullptr};
    RayTracingMaterial rayTracingMaterial_;
    Matrix4x4 transform_;
    Matrix4x4 invertTransform_;
};