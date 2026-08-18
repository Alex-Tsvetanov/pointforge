// synthetic.hpp: генератор на изкуствени данни.
//
// Съществува, за да няма демонстрацията и проверката нужда от външен набор от
// данни. Всичко се получава от зърно, така че две изпълнения дават един и същ
// вход, а измерване върху различен вход не е измерване.
//
// Сцената нарочно съдържа три различни по форма части. Само равнина не стига:
// върху равнина съвместяването е недоопределено по две транслации и по една
// ротация, и ICP „сходи“ към произволна точка от цяло семейство решения, което
// изглежда като успех, докато не се сравни с еталона.
#ifndef POINTFORGE_SYNTHETIC_HPP
#define POINTFORGE_SYNTHETIC_HPP

#include <cstddef>
#include <cstdint>

#include "pointforge/image.hpp"
#include "pointforge/point_cloud.hpp"

namespace pointforge {

struct SceneOptions {
    std::size_t plane_points = 4000;
    std::size_t sphere_points = 3000;
    std::size_t box_points = 3000;
    float plane_extent = 4.0F;   // половин страна на квадратната равнина
    float sphere_radius = 1.0F;
    Point3 sphere_center{-1.5F, 1.2F, 1.0F};
    Point3 box_center{1.6F, -1.0F, 0.8F};
    float box_half_size = 0.8F;
    std::uint32_t seed = 20260819U;
};

// Равнина z = height, точки, разхвърляни равномерно в квадрат.
PointCloud make_plane(std::size_t count, float half_extent, float height, std::uint32_t seed);

// Точки върху сферична повърхнина, равномерно по площ. Наивното теглене на
// сферични ъгли равномерно струпва точки около полюсите.
PointCloud make_sphere(std::size_t count, float radius, const Point3& center, std::uint32_t seed);

// Точки върху шестте стени на куб, равномерно по площ.
PointCloud make_box(std::size_t count, float half_size, const Point3& center, std::uint32_t seed);

// Равнина, сфера и куб в един облак.
PointCloud make_scene(const SceneOptions& options = {});

// Добавя независим гаусов шум по трите координати.
PointCloud add_gaussian_noise(const PointCloud& cloud, float sigma, std::uint32_t seed);

// Изхвърля дял от точките на случаен принцип. Използва се, за да имат двата
// облака различни точки, а не едни и същи, преместени: при еднакви точки ICP
// решава по-лека задача от истинската.
PointCloud random_subset(const PointCloud& cloud, double keep_fraction, std::uint32_t seed);

struct ImageSceneOptions {
    int width = 320;
    int height = 240;
    float noise_sigma = 3.0F;
    std::uint32_t seed = 20260819U;
};

// Изкуствено полутоново изображение: слаб наклон на фона, три правоъгълника,
// два кръга и гаусов шум. Формите са с различна яркост и различна площ, за да
// има какво да разграничи сегментацията.
Image8 make_test_image(const ImageSceneOptions& options = {});

}  // namespace pointforge

#endif  // POINTFORGE_SYNTHETIC_HPP
