#include "pointforge/image.hpp"

#include <cmath>
#include <limits>

namespace pointforge {

ImageF to_float(const Image8& image, int channel) {
    if (image.empty()) return {};
    if (channel < 0 || channel >= image.channels()) {
        throw std::invalid_argument("to_float: несъществуващ канал");
    }
    ImageF out(image.width(), image.height());
    for (int y = 0; y < image.height(); ++y) {
        for (int x = 0; x < image.width(); ++x) {
            out.at(x, y) = static_cast<float>(image.at(x, y, channel));
        }
    }
    return out;
}

Image8 to_image8(const ImageF& image) {
    if (image.empty()) return {};
    Image8 out(image.width(), image.height(), 1);
    for (int y = 0; y < image.height(); ++y) {
        for (int x = 0; x < image.width(); ++x) {
            const float v = std::clamp(image.at(x, y) + 0.5F, 0.0F, 255.0F);
            out.at(x, y) = static_cast<std::uint8_t>(v);
        }
    }
    return out;
}

Image8 to_grayscale(const Image8& image) {
    if (image.channels() == 1) return image;
    Image8 out(image.width(), image.height(), 1);
    for (int y = 0; y < image.height(); ++y) {
        for (int x = 0; x < image.width(); ++x) {
            // Тегла за яркост по ITU-R BT.601, същите, които ползва
            // преобразуването към Y в повечето кодеци.
            const float r = static_cast<float>(image.at(x, y, 0));
            const float g = static_cast<float>(image.at(x, y, 1));
            const float b = static_cast<float>(image.at(x, y, 2));
            const float y_value = 0.299F * r + 0.587F * g + 0.114F * b;
            out.at(x, y) = static_cast<std::uint8_t>(std::clamp(y_value + 0.5F, 0.0F, 255.0F));
        }
    }
    return out;
}

Image8 normalize_to_image8(const ImageF& image) {
    if (image.empty()) return {};
    float lo = std::numeric_limits<float>::infinity();
    float hi = -std::numeric_limits<float>::infinity();
    for (const float v : image.data()) {
        lo = std::min(lo, v);
        hi = std::max(hi, v);
    }
    // Постоянно изображение: мащабирането би било деление на нула, а
    // единствената смислена стойност е нула.
    const float span = hi - lo;
    Image8 out(image.width(), image.height(), 1);
    if (span <= 0.0F) return out;
    const float scale = 255.0F / span;
    for (int y = 0; y < image.height(); ++y) {
        for (int x = 0; x < image.width(); ++x) {
            const float v = (image.at(x, y) - lo) * scale;
            out.at(x, y) = static_cast<std::uint8_t>(std::clamp(v + 0.5F, 0.0F, 255.0F));
        }
    }
    return out;
}

}  // namespace pointforge
