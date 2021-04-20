#include "resource/model.h"

bool intersect(const Vec3f& ori, const Vec3f& dir, const Vec3f& min, const Vec3f& max) {
    float tmin = (min.x - ori.x) / dir.x;
    float tmax = (max.x - ori.x) / dir.x;

    if (tmin > tmax) std::swap(tmin, tmax);

    float tymin = (min.y - ori.y) / dir.y;
    float tymax = (max.y - ori.y) / dir.y;

    if (tymin > tymax) std::swap(tymin, tymax);

    if ((tmin > tymax) || (tymin > tmax)) return false;

    if (tymin > tmin) tmin = tymin;

    if (tymax < tmax) tmax = tymax;

    float tzmin = (min.z - ori.z) / dir.z;
    float tzmax = (max.z - ori.z) / dir.z;

    if (tzmin > tzmax) std::swap(tzmin, tzmax);

    if ((tmin > tzmax) || (tzmin > tmax)) return false;

    if (tzmin > tmin) tmin = tzmin;

    if (tzmax < tmax) tmax = tzmax;

    return true;
}

bool Model::ray_intersect(const Vec3f& ori, const Vec3f& dir, float& t0, float& u, float& v, Vec3f& hitNormal) const {
    const Vec3f localOri = projection<3>(embed<4>(ori) * invertTransform_);
    const Vec3f localDir = projection<3>(embed<4>(dir, 0.f) * invertTransform_).normalize();
    const auto& [boundMin, boundMax] = meshRes_->get_localbound();
    if (intersect(localOri, localDir, boundMin, boundMax)) {
        float t = std::numeric_limits<float>::max();
        int hitFace = -1;
        float u_, v_;
        for (int i = 0; i < meshRes_->nfaces(); ++i) {
            float tnear;
            float uTemp, vTemp;
            if (meshRes_->ray_triangle_intersect(i, localOri, localDir, tnear, uTemp, vTemp) && tnear < t) {
                t = tnear;
                u_ = uTemp;
                v_ = vTemp;
                hitFace = i;
            }
        }
        if (hitFace >= 0) {
            const Vec3f localHitPoint = localOri + localDir * t;
            const Vec3f hitPoint = projection<3>(embed<4>(localHitPoint) * transform_);
            t0 = ((hitPoint.x - ori.x) / dir.x);

            const auto& faceVertex = meshRes_->face_vertex(hitFace);
            const Vec3f& edge1 = meshRes_->vert(faceVertex[1]) - meshRes_->vert(faceVertex[0]);
            const Vec3f& edge2 = meshRes_->vert(faceVertex[2]) - meshRes_->vert(faceVertex[0]);
            hitNormal = cross(edge1, edge2);

            const auto& faceUV = meshRes_->face_uv(hitFace);
            if (faceUV[0] != -1 && materialRes_ != nullptr) {
                const Vec2f& uv0 = meshRes_->uv(faceUV[0]);
                const Vec2f& uv1 = meshRes_->uv(faceUV[1]);
                const Vec2f& uv2 = meshRes_->uv(faceUV[2]);
                const Vec2f& uv_ = uv1 * u_ + uv2 * v_ - uv0 * (u_ + v_ - 1);
                u = uv_.x;
                v = uv_.y;
                if (materialRes_ != nullptr) {
                    const auto& faceNormal = meshRes_->face_normal(hitFace);
                    if (faceNormal[0] != -1) {
                        hitNormal = meshRes_->normal(faceNormal[1]) * u_ + meshRes_->normal(faceNormal[2]) * v_ -
                                    meshRes_->normal(faceNormal[0]) * (u_ + v_ - 1);
                    }
                    Matrix<3, 3, float> a;
                    a[0] = edge1;
                    a[1] = edge2;
                    a[2] = hitNormal;

                    const Matrix<3, 3, float> ai = a.invert();
                    const Vec3f& i0 = ai * Vec3f(uv1.x - uv0.x, uv2.x - uv0.x, 0.f);
                    const Vec3f& j0 = ai * Vec3f(uv1.y - uv0.y, uv2.y - uv0.y, 0.f);
                    Matrix<3, 3, float> b;
                    b.set_column(0, i0);
                    b.set_column(1, j0);
                    b.set_column(2, hitNormal);

                    hitNormal = b * materialRes_->normal({u, v});
                }
            }
            hitNormal = projection<3>(embed<4>(hitNormal, 0.f) * transform_).normalize();
        }
        return hitFace >= 0;
    }
    return false;
}

RayTracingMaterial Model::get_ray_tracing_material(const float& u, const float& v) const {
    if (materialRes_ == nullptr) return rayTracingMaterial_;
    RayTracingMaterial ans{rayTracingMaterial_};
    ans.diffuse_ = materialRes_->diffuse({u, v}).to_linear();
    ans.specularExponent_ = materialRes_->specular({u, v});
    return ans;
}