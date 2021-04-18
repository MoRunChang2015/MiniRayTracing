#include "resource/sphere.h"

bool Sphere::ray_intersect(const Vec3f& ori, const Vec3f& dir, float& t0) const {
    Vec3f l = center_ - ori;
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