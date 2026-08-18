#include "pointforge/image_ops.hpp"

#include <cmath>
#include <stdexcept>

namespace pointforge {
namespace {

// Прилага едномерно ядро по хоризонтала. Границата се обработва със захващане
// към ръба, което не внася тъмна ивица, каквато би дало допълване с нула.
ImageF convolve_rows(const ImageF& image, const std::vector<float>& kernel) {
    const int radius = static_cast<int>(kernel.size() / 2);
    ImageF out(image.width(), image.height());
    for (int y = 0; y < image.height(); ++y) {
        for (int x = 0; x < image.width(); ++x) {
            float sum = 0.0F;
            for (int k = -radius; k <= radius; ++k) {
                sum += kernel[static_cast<std::size_t>(k + radius)] * image.clamped(x + k, y);
            }
            out.at(x, y) = sum;
        }
    }
    return out;
}

ImageF convolve_columns(const ImageF& image, const std::vector<float>& kernel) {
    const int radius = static_cast<int>(kernel.size() / 2);
    ImageF out(image.width(), image.height());
    for (int y = 0; y < image.height(); ++y) {
        for (int x = 0; x < image.width(); ++x) {
            float sum = 0.0F;
            for (int k = -radius; k <= radius; ++k) {
                sum += kernel[static_cast<std::size_t>(k + radius)] * image.clamped(x, y + k);
            }
            out.at(x, y) = sum;
        }
    }
    return out;
}

}  // namespace

std::vector<float> gaussian_kernel(float sigma) {
    if (sigma <= 0.0F) throw std::invalid_argument("gaussian_kernel: sigma <= 0");
    const int radius = std::max(1, static_cast<int>(std::ceil(3.0F * sigma)));
    std::vector<float> kernel(static_cast<std::size_t>(2 * radius + 1));
    const float denominator = 2.0F * sigma * sigma;
    float sum = 0.0F;
    for (int k = -radius; k <= radius; ++k) {
        const float value = std::exp(-static_cast<float>(k * k) / denominator);
        kernel[static_cast<std::size_t>(k + radius)] = value;
        sum += value;
    }
    // Нормирането е по действителната сума на отрязаното ядро, а не по
    // аналитичната константа. Иначе изгладеното изображение излиза системно
    // по-тъмно, толкова, колкото е отрязаната опашка.
    for (float& value : kernel) value /= sum;
    return kernel;
}

ImageF gaussian_blur(const ImageF& image, float sigma) {
    if (image.empty()) return {};
    const std::vector<float> kernel = gaussian_kernel(sigma);
    return convolve_columns(convolve_rows(image, kernel), kernel);
}

Gradient sobel(const ImageF& image) {
    Gradient result;
    if (image.empty()) return result;

    const std::vector<float> smooth = {1.0F, 2.0F, 1.0F};
    const std::vector<float> derive = {-1.0F, 0.0F, 1.0F};

    // gx: диференциране по x, изглаждане по y. gy: обратното.
    result.gx = convolve_columns(convolve_rows(image, derive), smooth);
    result.gy = convolve_columns(convolve_rows(image, smooth), derive);

    result.magnitude = ImageF(image.width(), image.height());
    for (int y = 0; y < image.height(); ++y) {
        for (int x = 0; x < image.width(); ++x) {
            const float dx = result.gx.at(x, y);
            const float dy = result.gy.at(x, y);
            result.magnitude.at(x, y) = std::sqrt(dx * dx + dy * dy);
        }
    }
    return result;
}

ImageF non_maximum_suppression(const Gradient& gradient) {
    const ImageF& magnitude = gradient.magnitude;
    if (magnitude.empty()) return {};

    ImageF out(magnitude.width(), magnitude.height());
    for (int y = 0; y < magnitude.height(); ++y) {
        for (int x = 0; x < magnitude.width(); ++x) {
            const float dx = gradient.gx.at(x, y);
            const float dy = gradient.gy.at(x, y);
            const float m = magnitude.at(x, y);
            if (m <= 0.0F) continue;

            // Закръгляване на посоката до едно от четири направления, направо
            // от знаците и отношението на компонентите. Изчисляването на
            // atan2 за всеки пиксел е чиста загуба: границите между секторите
            // отговарят на tan(22.5) и tan(67.5), а те се проверяват с две
            // умножения.
            const float ax = std::fabs(dx);
            const float ay = std::fabs(dy);
            int nx = 0;
            int ny = 0;
            if (ax >= 2.414214F * ay) {
                nx = 1;  // почти хоризонтален градиент, вертикален контур
            } else if (ay >= 2.414214F * ax) {
                ny = 1;
            } else if ((dx > 0.0F) == (dy > 0.0F)) {
                nx = 1;
                ny = 1;
            } else {
                nx = 1;
                ny = -1;
            }

            const float before = magnitude.clamped(x - nx, y - ny);
            const float after = magnitude.clamped(x + nx, y + ny);
            // Строго спрямо единия съсед и нестрого спрямо другия: при плато с
            // равни стойности строгото сравнение и от двете страни изтрива
            // целия контур, а нестрогото го оставя двоен.
            if (m > before && m >= after) out.at(x, y) = m;
        }
    }
    return out;
}

Image8 hysteresis_threshold(const ImageF& suppressed, float low, float high) {
    if (suppressed.empty()) return {};
    if (low > high) std::swap(low, high);

    const int width = suppressed.width();
    const int height = suppressed.height();
    Image8 out(width, height, 1);

    std::vector<std::pair<int, int>> stack;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (suppressed.at(x, y) >= high) {
                out.at(x, y) = 255;
                stack.emplace_back(x, y);
            }
        }
    }

    // Проследяване на слабите отговори от силните навън. Обхождането е с явен
    // стек, а не с рекурсия: при контур през цялото изображение дълбочината
    // става колкото броя на пикселите му.
    while (!stack.empty()) {
        const auto [x, y] = stack.back();
        stack.pop_back();
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) continue;
                const int nx = x + dx;
                const int ny = y + dy;
                if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
                if (out.at(nx, ny) != 0) continue;
                if (suppressed.at(nx, ny) < low) continue;
                out.at(nx, ny) = 255;
                stack.emplace_back(nx, ny);
            }
        }
    }
    return out;
}

Image8 threshold(const ImageF& image, float value, bool keep_above) {
    if (image.empty()) return {};
    Image8 out(image.width(), image.height(), 1);
    for (int y = 0; y < image.height(); ++y) {
        for (int x = 0; x < image.width(); ++x) {
            const bool above = image.at(x, y) >= value;
            out.at(x, y) = (above == keep_above) ? 255 : 0;
        }
    }
    return out;
}

}  // namespace pointforge
