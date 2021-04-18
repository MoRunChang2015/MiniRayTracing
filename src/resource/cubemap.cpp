#include "resource/cubemap.h"

CubeMap::CubeMap(const std::vector<std::string>& maps) {
    assert(maps.size() == 6);
    maps_.resize(6);
    for (size_t i = 0; i < 6; ++i) maps_[i].read_tga_file(maps[i].c_str());
}

Vec2f CubeMap::get_uv(const size_t& index, const Vec3f& dir) {
    Vec2f uv;
    float factor;
    switch (index) {
        case 0:  // front
            factor = 1.f / dir.x;
            uv.x = 1 + dir.z * factor;
            uv.y = 1 - dir.y * factor;
            break;
        case 1:  // up
            factor = 1.f / dir.y;
            uv.x = 1 + dir.x * factor;
            uv.y = 1 - dir.z * factor;
            break;
        case 2:  // right
            factor = 1.f / dir.z;
            uv.x = 1 - dir.x * factor;
            uv.y = 1 - dir.y * factor;
            break;
        case 3:  // back
            factor = 1.f / dir.x;
            uv.x = 1 + dir.z * factor;
            uv.y = 1 + dir.y * factor;
            break;
        case 4:  // down
            factor = 1.f / dir.y;
            uv.x = 1 + dir.z * factor;
            uv.y = 1 + dir.x * factor;
            break;
        case 5:  // left
            factor = 1.f / dir.z;
            uv.x = 1 - dir.x * factor;
            uv.y = 1 + dir.y * factor;
            break;
    }
    return uv / 2.f;
}

TGAColor CubeMap::sample(const Vec3f& dir) const {
    const size_t index = get_face(dir);
    const Vec2f uv = get_uv(index, dir);
    return maps_[index].get(static_cast<int>(uv.x * maps_[index].get_width() - 0.5),
                            static_cast<int>(uv.y * maps_[index].get_height() - 0.5));
}