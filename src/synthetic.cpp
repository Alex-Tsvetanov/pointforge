#include "pointforge/synthetic.hpp"

#include <algorithm>
#include <cmath>
#include <random>

namespace pointforge {
namespace {

// Едно и също устройство навсякъде, за да е повторяемостта свойство на
// проекта, а не съвпадение. std::mt19937 е определено от стандарта до бита,
// за разлика от разпределенията, чиито реализации се различават между
// библиотеките. Затова числата се теглят с generate_canonical и се превръщат
// на ръка, вместо с std::uniform_real_distribution.
float uniform01(std::mt19937& rng) {
    return static_cast<float>(std::generate_canonical<double, 32>(rng));
}

float uniform(std::mt19937& rng, float lo, float hi) { return lo + (hi - lo) * uniform01(rng); }

// Бокс-Мюлер. Двете стойности се връщат последователно, за да не се хаби
// втората, но състоянието се пази локално на извикващата функция.
struct NormalSource {
    std::mt19937& rng;
    bool has_spare = false;
    float spare = 0.0F;

    float next() {
        if (has_spare) {
            has_spare = false;
            return spare;
        }
        // Долната граница пази логаритъма от нула.
        const float u1 = std::max(uniform01(rng), 1e-7F);
        const float u2 = uniform01(rng);
        const float radius = std::sqrt(-2.0F * std::log(u1));
        const float angle = 6.2831853F * u2;
        spare = radius * std::sin(angle);
        has_spare = true;
        return radius * std::cos(angle);
    }
};

}  // namespace

PointCloud make_plane(std::size_t count, float half_extent, float height, std::uint32_t seed) {
    std::mt19937 rng(seed);
    PointCloud cloud;
    cloud.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        cloud.push_back(uniform(rng, -half_extent, half_extent),
                        uniform(rng, -half_extent, half_extent), height);
    }
    return cloud;
}

PointCloud make_sphere(std::size_t count, float radius, const Point3& center, std::uint32_t seed) {
    std::mt19937 rng(seed);
    PointCloud cloud;
    cloud.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        // Равномерно по площ: косинусът на полярния ъгъл е равномерен в
        // [-1, 1]. Тегленето на самия ъгъл равномерно струпва точки на
        // полюсите.
        const float cos_theta = uniform(rng, -1.0F, 1.0F);
        const float sin_theta = std::sqrt(std::max(0.0F, 1.0F - cos_theta * cos_theta));
        const float phi = uniform(rng, 0.0F, 6.2831853F);
        cloud.push_back(center.x + radius * sin_theta * std::cos(phi),
                        center.y + radius * sin_theta * std::sin(phi), center.z + radius * cos_theta);
    }
    return cloud;
}

PointCloud make_box(std::size_t count, float half_size, const Point3& center, std::uint32_t seed) {
    std::mt19937 rng(seed);
    PointCloud cloud;
    cloud.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        const int face = static_cast<int>(uniform(rng, 0.0F, 6.0F)) % 6;
        const float a = uniform(rng, -half_size, half_size);
        const float b = uniform(rng, -half_size, half_size);
        const float fixed = (face % 2 == 0) ? half_size : -half_size;
        Point3 p{};
        switch (face / 2) {
            case 0:
                p = {fixed, a, b};
                break;
            case 1:
                p = {a, fixed, b};
                break;
            default:
                p = {a, b, fixed};
                break;
        }
        cloud.push_back(center.x + p.x, center.y + p.y, center.z + p.z);
    }
    return cloud;
}

PointCloud make_scene(const SceneOptions& options) {
    // Различни зърна за отделните части, изведени от едно. Иначе трите части
    // излизат с една и съща последователност от числа и корелират по начин,
    // който се вижда в резултата.
    PointCloud cloud =
        make_plane(options.plane_points, options.plane_extent, 0.0F, options.seed);
    const PointCloud sphere =
        make_sphere(options.sphere_points, options.sphere_radius, options.sphere_center,
                    options.seed + 1U);
    const PointCloud box =
        make_box(options.box_points, options.box_half_size, options.box_center, options.seed + 2U);

    cloud.reserve(cloud.size() + sphere.size() + box.size());
    for (std::size_t i = 0; i < sphere.size(); ++i) cloud.push_back(sphere.point(i));
    for (std::size_t i = 0; i < box.size(); ++i) cloud.push_back(box.point(i));
    return cloud;
}

PointCloud add_gaussian_noise(const PointCloud& cloud, float sigma, std::uint32_t seed) {
    if (sigma <= 0.0F) return cloud;
    std::mt19937 rng(seed);
    NormalSource normal{rng};
    PointCloud out;
    out.resize(cloud.size());
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        out.xs()[i] = cloud.xs()[i] + sigma * normal.next();
        out.ys()[i] = cloud.ys()[i] + sigma * normal.next();
        out.zs()[i] = cloud.zs()[i] + sigma * normal.next();
    }
    return out;
}

PointCloud random_subset(const PointCloud& cloud, double keep_fraction, std::uint32_t seed) {
    keep_fraction = std::clamp(keep_fraction, 0.0, 1.0);
    std::mt19937 rng(seed);
    PointCloud out;
    out.reserve(static_cast<std::size_t>(cloud.size() * keep_fraction) + 1);
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        if (uniform01(rng) <= keep_fraction) out.push_back(cloud.point(i));
    }
    return out;
}

Image8 make_test_image(const ImageSceneOptions& options) {
    Image8 image(options.width, options.height, 1);
    std::mt19937 rng(options.seed);
    NormalSource normal{rng};

    const float w = static_cast<float>(options.width);
    const float h = static_cast<float>(options.height);

    struct Rect {
        int x0;
        int y0;
        int x1;
        int y1;
        float value;
    };
    const Rect rects[] = {
        {static_cast<int>(0.08F * w), static_cast<int>(0.12F * h), static_cast<int>(0.32F * w),
         static_cast<int>(0.45F * h), 210.0F},
        {static_cast<int>(0.55F * w), static_cast<int>(0.10F * h), static_cast<int>(0.90F * w),
         static_cast<int>(0.30F * h), 160.0F},
        {static_cast<int>(0.40F * w), static_cast<int>(0.60F * h), static_cast<int>(0.62F * w),
         static_cast<int>(0.88F * h), 235.0F},
    };

    struct Disc {
        float cx;
        float cy;
        float radius;
        float value;
    };
    // Кръговете стоят достатъчно далече от правоъгълниците, за да не се слеят
    // при изглаждане: сегментацията след праг би върнала една област вместо
    // две и проверката би отчела грешка, която е в генератора, не в кода.
    const Disc discs[] = {
        {0.80F * w, 0.68F * h, 0.11F * std::min(w, h), 190.0F},
        {0.16F * w, 0.76F * h, 0.075F * std::min(w, h), 130.0F},
    };

    for (int y = 0; y < options.height; ++y) {
        for (int x = 0; x < options.width; ++x) {
            // Слаб наклон на фона. Постоянният фон прави прага тривиален и
            // сегментацията изглежда по-добра, отколкото е.
            float value = 30.0F + 25.0F * (static_cast<float>(x) / w);

            for (const Rect& r : rects) {
                if (x >= r.x0 && x < r.x1 && y >= r.y0 && y < r.y1) value = r.value;
            }
            for (const Disc& d : discs) {
                const float dx = static_cast<float>(x) - d.cx;
                const float dy = static_cast<float>(y) - d.cy;
                if (dx * dx + dy * dy <= d.radius * d.radius) value = d.value;
            }

            if (options.noise_sigma > 0.0F) value += options.noise_sigma * normal.next();
            image.at(x, y) = static_cast<std::uint8_t>(std::clamp(value + 0.5F, 0.0F, 255.0F));
        }
    }
    return image;
}

}  // namespace pointforge
