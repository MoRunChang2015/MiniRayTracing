#include "resource/material.h"
#include <iostream>
#include <string>

void load_texture(const std::string& filename, TGAImage& out) {
    std::cout << "Texture file " << filename << " loading " << (out.read_tga_file(filename.c_str()) ? " ok " : " fail ")
              << std::endl;
}

Material::Material(const std::string& diffuseFile, const std::string& normalFile, const std::string& specularFile) {
    load_texture(diffuseFile, diffuseMap_);
    diffuseMap_.flip_vertically();
    if (!normalFile.empty()) {
        load_texture(normalFile, normalMap_);
        normalMap_.flip_vertically();
    }
    if (!specularFile.empty()) {
        load_texture(specularFile, specularMap_);
        specularMap_.flip_vertically();
    }
}

TGAColor Material::diffuse(Vec2f uv) {
    Vec2i uvi(static_cast<int>(uv[0] * diffuseMap_.get_width()), static_cast<int>(uv[1] * diffuseMap_.get_height()));
    return diffuseMap_.get(uvi[0], uvi[1]);
}
Vec3f Material::normal(Vec2f uv) {
    Vec2i uvi(static_cast<int>(uv[0] * normalMap_.get_width()), static_cast<int>(uv[1] * normalMap_.get_height()));
    TGAColor c = normalMap_.get(uvi[0], uvi[1]);
    Vec3f ret;
    for (size_t i = 0; i < 3; ++i) {
        ret[2 - i] = static_cast<float>(c[i] / 255.0f * 2.f - 1.f);
    }
    return ret;
}
float Material::specular(Vec2f uv) {
    Vec2i uvi(static_cast<int>(uv[0] * specularMap_.get_width()), static_cast<int>(uv[1] * specularMap_.get_height()));
    return specularMap_.get(uvi[0], uvi[1])[0] / 1.0f;
}