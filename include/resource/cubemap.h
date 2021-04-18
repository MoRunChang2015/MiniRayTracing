#pragma once
#include <vector>

#include "util/tgaImage.h"

class CubeMap {
   public:
    // Front, Up, Right, Back, Down, Left
    CubeMap(const std::vector<std::string>& maps);

    TGAColor sample(const Vec3f& dir) const;

   private:
    /*
        front   up  right    back    down   left
        +x      +y  +z      -x      -y      -z
        0       1   2       3       4       5
    */
    static size_t get_face(const Vec3f& dir) {
        size_t m = 0;
        for (size_t i = 1; i < 3; ++i)
            if (std::fabs(dir[i]) > std::fabs(dir[m])) m = i;
        return m + (dir[m] > 0 ? 0 : 3);
    }

   static Vec2f get_uv(const size_t& index, const Vec3f& dir);

    std::vector<TGAImage> maps_;
};
