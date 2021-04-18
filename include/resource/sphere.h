#pragma once
#include "util/geometry.h"
#include "material.h"

struct Sphere {
    Vec3f center_;
    float radius_;
    RayTracingMaterial material_;

    Sphere(const Vec3f& c, const float& r) : center_(c), radius_(r){};
    Sphere(const Vec3f& c, const float& r, const RayTracingMaterial& m) : center_(c), radius_(r), material_(m){};

    // dir is normalized
    bool ray_intersect(const Vec3f& org, const Vec3f& dir, float& t0) const;
};