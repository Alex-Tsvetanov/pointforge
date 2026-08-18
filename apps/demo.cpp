// demo.cpp: една команда, която пуска целия конвейер и показва какво излиза.
//
// Първата половина е геометричната: сцена, известно кораво преобразувание,
// шум, прореждане, индекс и съвместяване. Отпечатва се възстановеното
// преобразувание до истинското, за да се вижда не само че процедурата е
// сходила, а и към какво.
//
// Втората половина е растерната: изкуствено изображение, изглаждане, градиент,
// потискане на немаксимумите, двупрагово решение и сегментация. Резултатите се
// записват като PGM и PPM.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <string>

#include "pointforge/cloud_io.hpp"
#include "pointforge/icp.hpp"
#include "pointforge/image_io.hpp"
#include "pointforge/image_ops.hpp"
#include "pointforge/labeling.hpp"
#include "pointforge/synthetic.hpp"
#include "pointforge/text.hpp"
#include "pointforge/timing.hpp"
#include "pointforge/voxel_grid.hpp"

using namespace pointforge;

namespace {

void print_rule(const std::string& title) {
    const std::size_t length = utf8_length(title);
    std::cout << "\n== " << title << " " << std::string(length < 66 ? 66 - length : 3, '=') << "\n";
}

void print_transform(const std::string& label, const RigidTransform& t) {
    std::printf("  %s\n", label.c_str());
    for (int i = 0; i < 3; ++i) {
        std::printf("    [ %9.6f %9.6f %9.6f ]   [ %9.6f ]\n", t.rotation(i, 0), t.rotation(i, 1),
                    t.rotation(i, 2),
                    i == 0   ? static_cast<double>(t.translation.x)
                    : i == 1 ? static_cast<double>(t.translation.y)
                             : static_cast<double>(t.translation.z));
    }
}

std::string argument_value(int argc, char** argv, const std::string& name,
                           const std::string& fallback) {
    for (int i = 1; i + 1 < argc; ++i) {
        if (name == argv[i]) return argv[i + 1];
    }
    return fallback;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string output_dir = argument_value(argc, argv, "--output-dir", "demo-output");
    std::error_code ec;
    std::filesystem::create_directories(output_dir, ec);
    if (ec) {
        std::cerr << "не може да се създаде директория " << output_dir << ": " << ec.message()
                  << "\n";
        return 1;
    }
    const std::filesystem::path out(output_dir);

    std::cout << "PointForge, демонстрация\n";
    std::cout << "изход: " << std::filesystem::absolute(out).string() << "\n";

    // ---------------------------------------------------------------------
    // 1. Данни
    // ---------------------------------------------------------------------
    print_rule("1. Изкуствени данни");

    SceneOptions scene_options;
    scene_options.plane_points = 30000;
    scene_options.sphere_points = 20000;
    scene_options.box_points = 20000;
    const PointCloud reference = make_scene(scene_options);

    const float noise_sigma = 0.004F;
    RigidTransform truth;
    truth.rotation = rotation_axis_angle(0.31, -0.57, 0.76, 0.13);  // около 7.4 градуса
    truth.translation = {0.180F, -0.120F, 0.075F};

    // Целевият облак е зашумено копие на еталона. Изходният е ДРУГА извадка от
    // същата сцена, също зашумена, преместена обратно на истинското
    // преобразувание. Така двата облака нямат общи точки и задачата е тази,
    // която ICP решава на практика.
    const PointCloud target = add_gaussian_noise(reference, noise_sigma, 101U);
    const PointCloud source = transform_cloud(
        add_gaussian_noise(random_subset(reference, 0.75, 202U), noise_sigma, 303U),
        truth.inverse());

    std::printf("  еталонна сцена      : %zu точки\n", reference.size());
    std::printf("  целеви облак        : %zu точки, шум sigma = %.4f\n", target.size(),
                static_cast<double>(noise_sigma));
    std::printf("  изходен облак       : %zu точки (75 %% извадка, преместена)\n", source.size());

    // ---------------------------------------------------------------------
    // 2. Прореждане
    // ---------------------------------------------------------------------
    print_rule("2. Прореждане с вокселна решетка");

    const float leaf = 0.05F;
    VoxelGridStats target_stats;
    VoxelGridStats source_stats;
    const PointCloud target_small = voxel_downsample(target, leaf, &target_stats);
    const PointCloud source_small = voxel_downsample(source, leaf, &source_stats);
    const VoxelGridShape shape = voxel_grid_shape(target, leaf);

    std::printf("  ребро на кубчето    : %.3f\n", static_cast<double>(leaf));
    std::printf("  решетка             : %llu x %llu x %llu кубчета\n",
                static_cast<unsigned long long>(shape.nx), static_cast<unsigned long long>(shape.ny),
                static_cast<unsigned long long>(shape.nz));
    std::printf("  целеви  : %zu -> %zu точки (%.1f %% остават)\n", target_stats.input_points,
                target_stats.output_points, 100.0 * target_stats.reduction_ratio());
    std::printf("  изходен : %zu -> %zu точки (%.1f %% остават)\n", source_stats.input_points,
                source_stats.output_points, 100.0 * source_stats.reduction_ratio());

    // ---------------------------------------------------------------------
    // 3. Индекс
    // ---------------------------------------------------------------------
    print_rule("3. Пространствен индекс");

    KdTree index;
    const TimingStats build_time =
        measure(5, 1, [&] { index.build(target_small, KdTreeOptions{32}); });

    std::printf("  точки в индекса     : %zu\n", index.size());
    std::printf("  възли               : %zu, точки в лист до %u\n", index.node_count(),
                index.leaf_size());
    std::printf("  памет               : %.2f MiB\n",
                static_cast<double>(index.memory_bytes()) / (1024.0 * 1024.0));
    std::printf("  време за построяване: %.3f ms (медиана от %d)\n", build_time.median_ms,
                build_time.repetitions);

    // Проверка на индекса срещу изчерпателно търсене върху малка извадка.
    // Твърдението „индексът е точен“ струва двадесет реда, а без тях остава
    // твърдение.
    std::size_t checked = 0;
    std::size_t agreed = 0;
    for (std::size_t i = 0; i < target_small.size(); i += target_small.size() / 50 + 1) {
        const Point3 query = target_small.point(i);
        const std::vector<Neighbor> expected = brute_force_knn(target_small, query, 10);
        const std::vector<Neighbor> got = index.knn(query, 10);
        bool same = expected.size() == got.size();
        for (std::size_t j = 0; same && j < got.size(); ++j) {
            same = std::fabs(expected[j].squared_distance - got[j].squared_distance) < 1e-6F;
        }
        ++checked;
        agreed += same ? 1 : 0;
    }
    std::printf("  съвпадение с изчерпателно търсене: %zu от %zu заявки\n", agreed, checked);

    // ---------------------------------------------------------------------
    // 4. Съвместяване
    // ---------------------------------------------------------------------
    print_rule("4. Съвместяване, точка към точка");

    IcpOptions icp_options;
    icp_options.max_iterations = 60;
    icp_options.max_correspondence_distance = 0.5F;
    icp_options.path = NnPath::Batched;

    IcpResult result;
    const TimingStats icp_time = measure(3, 1, [&] {
        result = icp_point_to_point(source_small, target_small, index, icp_options);
    });

    print_transform("възстановено преобразувание:", result.transform);
    print_transform("истинско преобразувание:", truth);

    const Mat3 relative = result.transform.rotation * truth.rotation.transposed();
    const Point3 dt = result.transform.translation - truth.translation;
    const double translation_error =
        std::sqrt(static_cast<double>(dt.x) * dt.x + static_cast<double>(dt.y) * dt.y +
                  static_cast<double>(dt.z) * dt.z);

    std::printf("\n  итерации            : %d%s\n", result.iterations,
                result.converged ? " (сходимост)" : " (изчерпан лимит)");
    std::printf("  приети съответствия : %zu от %zu\n", result.correspondences, source_small.size());
    std::printf("  крайно отклонение   : %.6f (шумът в данните е %.4f)\n", result.rmse,
                static_cast<double>(noise_sigma));
    std::printf("  грешка по ъгъл      : %.6f градуса\n", rotation_angle(relative) * 180.0 / 3.14159265358979);
    std::printf("  грешка по транслация: %.6f\n", translation_error);
    std::printf("  време               : %.2f ms (медиана от %d)\n", icp_time.median_ms,
                icp_time.repetitions);

    const PointCloud registered = transform_cloud(source_small, result.transform);
    write_ply((out / "target.ply").string(), target_small);
    write_ply((out / "source.ply").string(), source_small);
    write_ply((out / "registered.ply").string(), registered);
    std::printf("  записани            : target.ply, source.ply, registered.ply\n");

    // ---------------------------------------------------------------------
    // 5. Контури
    // ---------------------------------------------------------------------
    print_rule("5. Откриване на контури");

    ImageSceneOptions image_options;
    image_options.width = 320;
    image_options.height = 240;
    image_options.noise_sigma = 6.0F;
    const Image8 scene_image = make_test_image(image_options);

    const ImageF grey = to_float(scene_image);
    const ImageF blurred = gaussian_blur(grey, 1.6F);
    const Gradient gradient = sobel(blurred);
    const ImageF thin = non_maximum_suppression(gradient);
    const Image8 edges = hysteresis_threshold(thin, 40.0F, 110.0F);

    std::size_t edge_pixels = 0;
    for (const std::uint8_t v : edges.data()) edge_pixels += (v != 0) ? 1 : 0;

    std::printf("  изображение         : %d x %d, шум sigma = %.1f\n", scene_image.width(),
                scene_image.height(), static_cast<double>(image_options.noise_sigma));
    std::printf("  гаусово ядро        : %zu отчета при sigma = 1.6\n", gaussian_kernel(1.6F).size());
    std::printf("  пиксели по контур   : %zu (%.2f %% от кадъра)\n", edge_pixels,
                100.0 * static_cast<double>(edge_pixels) / static_cast<double>(edges.pixel_count()));

    write_pgm((out / "01_input.pgm").string(), scene_image);
    write_pgm((out / "02_blurred.pgm").string(), to_image8(blurred));
    write_pgm((out / "03_gradient.pgm").string(), normalize_to_image8(gradient.magnitude));
    write_pgm((out / "04_suppressed.pgm").string(), normalize_to_image8(thin));
    write_pgm((out / "05_edges.pgm").string(), edges);
    std::printf("  записани            : 01_input.pgm .. 05_edges.pgm\n");

    // ---------------------------------------------------------------------
    // 6. Сегментация
    // ---------------------------------------------------------------------
    print_rule("6. Сегментация по свързани компоненти");

    const Image8 mask = threshold(blurred, 110.0F);
    const LabelResult labels = connected_components(mask, Connectivity::Eight, 50);

    std::printf("  праг                : 110, минимална площ 50 пиксела\n");
    std::printf("  намерени области    : %zu\n\n", labels.components.size());
    std::printf("    %s%s%s%s%s%s\n", pad_right("номер", 8).c_str(), pad_left("площ", 8).c_str(),
                pad_left("ширина", 10).c_str(), pad_left("височина", 11).c_str(),
                pad_left("център x", 11).c_str(), pad_left("център y", 11).c_str());
    for (const Component& component : labels.components) {
        std::printf("    %-8u%8u%10d%11d%11.1f%11.1f\n", component.label, component.area,
                    component.width(), component.height(), component.centroid_x,
                    component.centroid_y);
    }

    write_pgm((out / "06_mask.pgm").string(), mask);
    write_ppm((out / "07_segments.ppm").string(), colorize_labels(labels));
    std::printf("\n  записани            : 06_mask.pgm, 07_segments.ppm\n");

    print_rule("готово");
    return 0;
}
