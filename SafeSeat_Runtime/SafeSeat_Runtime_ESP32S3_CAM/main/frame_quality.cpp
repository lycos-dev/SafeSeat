#include "frame_quality.hpp"
#include <cstdint>

namespace {
inline int luma(const uint8_t *p) {
    return (77 * static_cast<int>(p[0]) + 150 * static_cast<int>(p[1]) + 29 * static_cast<int>(p[2])) >> 8;
}
}

float safeseat_rgb888_sharpness(const uint8_t *rgb, int width, int height) {
    if (!rgb || width < 8 || height < 8) return 0.0f;
    double sum = 0.0;
    uint32_t count = 0;
    for (int y = 2; y < height - 2; y += 2) {
        for (int x = 2; x < width - 2; x += 2) {
            const uint8_t *c = rgb + 3 * (y * width + x);
            const uint8_t *r = rgb + 3 * (y * width + (x + 2));
            const uint8_t *d = rgb + 3 * ((y + 2) * width + x);
            const int yc = luma(c), yr = luma(r), yd = luma(d);
            const int dx = yr - yc, dy = yd - yc;
            sum += static_cast<double>(dx * dx + dy * dy);
            ++count;
        }
    }
    return count ? static_cast<float>(sum / static_cast<double>(count)) : 0.0f;
}
