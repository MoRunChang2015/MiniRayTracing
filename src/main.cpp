#include "util/tgaImage.h"

const int kWidth = 800;
const int kHeight = 600;
const std::string kFrameBufferOutput = "./output.tga";

void render(TGAImage& frameBuffer) {
    for (size_t i = 0; i < kWidth; ++i)
        for (size_t j = 0; j < kHeight; ++j)
            frameBuffer.set(i, j, TGAColor(std::round(255.0 * i / kWidth), std::round(255.0 * j / kHeight), 0, 255));
}

int main() {
    TGAImage frameBuffer(kWidth, kHeight, TGAImage::RGB);
    render(frameBuffer);
    frameBuffer.flip_vertically();
    frameBuffer.write_tga_file(kFrameBufferOutput.c_str());
    return 0;
}