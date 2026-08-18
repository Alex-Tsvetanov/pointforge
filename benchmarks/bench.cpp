// bench.cpp: измерванията, които попълват таблиците в отчета.
//
// Няма външна библиотека за измерване. Часовникът е steady_clock, всяко
// измерване има загряващи изпълнения и се повтаря.
//
// Отчита се МИНИМУМЪТ от повторенията, а медианата и разсейването стоят до
// него като мярка за качеството на измерването. Изборът е следствие от
// наблюдение: на тази машина разсейването между повторенията стига стотици
// проценти, тоест има чужда работа по ядрата. Външната намеса може само да
// удължи изпълнението, никога да го скъси, затова минимумът е най-близкото до
// ненарушеното време, а медианата се движи с товара и не е повторяема. Когато
// разсейването е малко, двете съвпадат и изборът няма значение.
//
// Всеки ред от изхода е и ред от таблица: програмата пише и CSV, за да не се
// преписват числа на ръка.
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "pointforge/icp.hpp"
#include "pointforge/image_ops.hpp"
#include "pointforge/labeling.hpp"
#include "pointforge/synthetic.hpp"
#include "pointforge/text.hpp"
#include "pointforge/timing.hpp"
#include "pointforge/voxel_grid.hpp"

using namespace pointforge;

namespace {

struct Csv {
    std::ofstream file;
    explicit Csv(const std::string& path) : file(path, std::ios::binary) {}
    void row(const std::string& line) {
        if (file) file << line << "\n";
    }
};

PointCloud uniform_cloud(std::size_t count, std::uint32_t seed) {
    std::mt19937 rng(seed);
    PointCloud cloud;
    cloud.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        const auto draw = [&] {
            return static_cast<float>(std::generate_canonical<double, 32>(rng) * 10.0 - 5.0);
        };
        cloud.push_back(draw(), draw(), draw());
    }
    return cloud;
}

PointCloud query_points(std::size_t count, std::uint32_t seed) { return uniform_cloud(count, seed); }

// Заглавен ред на таблица. Подравняването минава през pad_left, защото
// спецификаторите за ширина на printf броят байтове, а кирилицата е по два.
void header(std::initializer_list<std::pair<const char*, std::size_t>> columns) {
    std::string line;
    for (const auto& [text, width] : columns) {
        line += pad_left(text, width);
        line += " ";
    }
    std::printf("%s\n", line.c_str());
}

void section(const char* title) {
    std::printf("\n%s\n", title);
    std::printf("%s\n", std::string(78, '-').c_str());
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
    const std::string csv_path = argument_value(argc, argv, "--csv", "pointforge_results.csv");
    Csv csv(csv_path);
    csv.row("group,case,size,parameter,metric,value,unit");

    std::printf("PointForge, измервания\n");
    std::printf("компилатор: ");
#if defined(__clang__)
    std::printf("clang %d.%d.%d\n", __clang_major__, __clang_minor__, __clang_patchlevel__);
#elif defined(__GNUC__)
    std::printf("g++ %d.%d.%d\n", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
#else
    std::printf("неизвестен\n");
#endif
    std::printf("резултати в CSV: %s\n", csv_path.c_str());

    const std::vector<std::size_t> sizes = {10000, 50000, 200000, 1000000};
    const std::size_t query_count = 20000;
    const PointCloud queries = query_points(query_count, 555U);

    // Сравнението между двата пътя е основното твърдение на работата, а
    // разликата между тях е от порядъка на разсейването при три повторения.
    // Затова точно тези измервания се повтарят повече пъти.
    constexpr int kQueryReps = 9;
    constexpr int kQueryWarmup = 2;

    // -----------------------------------------------------------------
    // 1. Построяване на индекса
    // -----------------------------------------------------------------
    section("1. Построяване на k-d дървото");
    header({{"точки", 12}, {"минимум, ms", 14}, {"медиана, ms", 14}, {"разсейване, %", 14},
            {"памет, MiB", 12}, {"възли", 10}});
    std::vector<PointCloud> clouds;
    std::vector<KdTree> trees;
    clouds.reserve(sizes.size());
    trees.reserve(sizes.size());

    for (const std::size_t n : sizes) {
        clouds.push_back(uniform_cloud(n, 1234U));
        KdTree tree;
        const TimingStats stats =
            measure(5, 1, [&] { tree.build(clouds.back(), KdTreeOptions{32}); });
        std::printf("%12zu %14.3f %14.3f %14.1f %12.2f %10zu\n", n, stats.min_ms, stats.median_ms,
                    100.0 * stats.spread_ratio(),
                    static_cast<double>(tree.memory_bytes()) / (1024.0 * 1024.0), tree.node_count());
        csv.row("build,kdtree," + std::to_string(n) + ",leaf32,min_ms," +
                std::to_string(stats.min_ms) + ",ms");
        csv.row("build,kdtree," + std::to_string(n) + ",leaf32,median_ms," +
                std::to_string(stats.median_ms) + ",ms");
        csv.row("build,kdtree," + std::to_string(n) + ",leaf32,spread_pct," +
                std::to_string(100.0 * stats.spread_ratio()) + ",%");
        csv.row("build,kdtree," + std::to_string(n) + ",leaf32,memory_mib," +
                std::to_string(static_cast<double>(tree.memory_bytes()) / (1024.0 * 1024.0)) +
                ",MiB");
        trees.push_back(std::move(tree));
    }

    // -----------------------------------------------------------------
    // 2. Пропускателна способност на заявката, скаларен срещу пакетен път
    // -----------------------------------------------------------------
    section("2. Заявка за k най-близки съседи, скаларен срещу пакетен път");
    header({{"точки", 12}, {"k", 4}, {"скаларен мин", 14}, {"пакетен мин", 14}, {"отношение", 10},
            {"разсейване, %", 14}, {"заявки/s (пак.)", 14}});

    for (std::size_t i = 0; i < sizes.size(); ++i) {
        const KdTree& tree = trees[i];
        for (const std::size_t k : {std::size_t{1}, std::size_t{8}, std::size_t{32}}) {
            std::vector<Neighbor> scratch;
            const auto sweep = [&](NnPath path) {
                return [&, path] {
                    for (std::size_t q = 0; q < queries.size(); ++q) {
                        tree.knn(queries.point(q), k, scratch, path);
                        keep(scratch.size());
                    }
                };
            };
            const TimingStats scalar = measure(kQueryReps, kQueryWarmup, sweep(NnPath::Scalar));
            const TimingStats batched = measure(kQueryReps, kQueryWarmup, sweep(NnPath::Batched));
            const double ratio = batched.min_ms > 0.0 ? scalar.min_ms / batched.min_ms : 0.0;
            const double throughput =
                batched.min_ms > 0.0 ? static_cast<double>(query_count) / (batched.min_ms / 1000.0)
                                        : 0.0;
            std::printf("%12zu %4zu %14.2f %14.2f %10.3f %14.1f %14.0f\n", sizes[i], k,
                        scalar.min_ms, batched.min_ms, ratio,
                        100.0 * std::max(scalar.spread_ratio(), batched.spread_ratio()), throughput);

            const std::string prefix =
                "knn,k" + std::to_string(k) + "," + std::to_string(sizes[i]) + ",";
            csv.row(prefix + "scalar,min_ms," + std::to_string(scalar.min_ms) + ",ms");
            csv.row(prefix + "batched,min_ms," + std::to_string(batched.min_ms) + ",ms");
            csv.row(prefix + "batched,spread_pct," +
                    std::to_string(100.0 * batched.spread_ratio()) + ",%");
            csv.row(prefix + "batched,queries_per_s," + std::to_string(throughput) + ",1/s");
        }
    }

    // -----------------------------------------------------------------
    // 3. Заявка по радиус
    // -----------------------------------------------------------------
    section("3. Заявка по радиус, скаларен срещу пакетен път");
    header({{"точки", 12}, {"радиус", 8}, {"скаларен мин", 14}, {"пакетен мин", 14},
            {"отношение", 10}, {"средно съседи", 12}});

    for (std::size_t i = 0; i < sizes.size(); ++i) {
        const KdTree& tree = trees[i];
        for (const float radius : {0.2F, 0.5F}) {
            std::vector<Neighbor> scratch;
            std::size_t found = 0;
            const auto sweep = [&](NnPath path) {
                return [&, path] {
                    found = 0;
                    for (std::size_t q = 0; q < 2000; ++q) {
                        tree.radius_search(queries.point(q), radius, scratch, path);
                        found += scratch.size();
                    }
                    keep(found);
                };
            };
            const TimingStats scalar = measure(kQueryReps, kQueryWarmup, sweep(NnPath::Scalar));
            const TimingStats batched = measure(kQueryReps, kQueryWarmup, sweep(NnPath::Batched));
            const double ratio = batched.min_ms > 0.0 ? scalar.min_ms / batched.min_ms : 0.0;
            std::printf("%12zu %8.2f %14.2f %14.2f %10.3f %12.1f\n", sizes[i],
                        static_cast<double>(radius), scalar.min_ms, batched.min_ms, ratio,
                        static_cast<double>(found) / 2000.0);

            const std::string prefix = "radius,r" + std::to_string(radius) + "," +
                                       std::to_string(sizes[i]) + ",";
            csv.row(prefix + "scalar,min_ms," + std::to_string(scalar.min_ms) + ",ms");
            csv.row(prefix + "batched,min_ms," + std::to_string(batched.min_ms) + ",ms");
        }
    }

    // -----------------------------------------------------------------
    // 4. Размер на листа
    // -----------------------------------------------------------------
    section("4. Влияние на размера на листа, 200000 точки, k = 8");
    header({{"лист", 12}, {"построяване мин", 15}, {"скаларен мин", 14}, {"пакетен мин", 14},
            {"отношение", 10}});
    {
        const PointCloud& cloud = clouds[2];
        for (const std::uint32_t leaf : {std::uint32_t{4}, std::uint32_t{8}, std::uint32_t{16},
                                         std::uint32_t{32}, std::uint32_t{64}, std::uint32_t{128}}) {
            KdTree tree;
            const TimingStats build = measure(3, 1, [&] { tree.build(cloud, KdTreeOptions{leaf}); });
            std::vector<Neighbor> scratch;
            const auto sweep = [&](NnPath path) {
                return [&, path] {
                    for (std::size_t q = 0; q < 5000; ++q) {
                        tree.knn(queries.point(q), 8, scratch, path);
                        keep(scratch.size());
                    }
                };
            };
            const TimingStats scalar = measure(kQueryReps, kQueryWarmup, sweep(NnPath::Scalar));
            const TimingStats batched = measure(kQueryReps, kQueryWarmup, sweep(NnPath::Batched));
            const double ratio = batched.min_ms > 0.0 ? scalar.min_ms / batched.min_ms : 0.0;
            std::printf("%12u %14.2f %14.2f %14.2f %10.3f\n", leaf, build.min_ms,
                        scalar.min_ms, batched.min_ms, ratio);
            const std::string prefix = "leaf,leaf" + std::to_string(leaf) + ",200000,";
            csv.row(prefix + "scalar,min_ms," + std::to_string(scalar.min_ms) + ",ms");
            csv.row(prefix + "batched,min_ms," + std::to_string(batched.min_ms) + ",ms");
            csv.row(prefix + "build,min_ms," + std::to_string(build.min_ms) + ",ms");
        }
    }

    // -----------------------------------------------------------------
    // 5. Изолиран обхождащ цикъл на лист
    // -----------------------------------------------------------------
    // Мерките в раздели 2 и 3 са за цялата заявка, а в нея обхождането на
    // дървото и купчината с кандидати заемат част от времето, която не зависи
    // от избрания път. Тук дървото е нарочно построено с един-единствен лист,
    // тоест размерът на листа е над броя на точките. Тогава всяка заявка е
    // само обхождане на лист и разликата между двата пътя е видима без
    // примеси. Цената е, че това вече не е заявка, каквато някой би направил
    // на практика, затова разделът стои отделно, а не заменя раздел 2.
    section("5. Изолиран обхождащ цикъл на лист (дърво от един лист)");
    header({{"точки в листа", 14}, {"k", 4}, {"скаларен мин", 14}, {"пакетен мин", 14},
            {"отношение", 10}, {"разсейване, %", 14}});
    {
        for (const std::size_t leaf_points : {std::size_t{1024}, std::size_t{8192},
                                              std::size_t{65536}}) {
            const PointCloud cloud = uniform_cloud(leaf_points, 4242U);
            KdTree tree(cloud, KdTreeOptions{static_cast<std::uint32_t>(leaf_points)});
            const std::size_t sweep_queries = 20000000 / leaf_points;

            for (const std::size_t k : {std::size_t{1}, std::size_t{8}}) {
                std::vector<Neighbor> scratch;
                const auto sweep = [&](NnPath path) {
                    return [&, path] {
                        for (std::size_t q = 0; q < sweep_queries; ++q) {
                            tree.knn(queries.point(q % queries.size()), k, scratch, path);
                            keep(scratch.size());
                        }
                    };
                };
                const TimingStats scalar = measure(kQueryReps, kQueryWarmup, sweep(NnPath::Scalar));
                const TimingStats batched = measure(kQueryReps, kQueryWarmup, sweep(NnPath::Batched));
                const double ratio =
                    batched.min_ms > 0.0 ? scalar.min_ms / batched.min_ms : 0.0;
                std::printf("%14zu %4zu %14.2f %14.2f %10.3f %14.1f\n", leaf_points, k,
                            scalar.min_ms, batched.min_ms, ratio,
                            100.0 * batched.spread_ratio());

                const std::string prefix = "leafscan,k" + std::to_string(k) + "," +
                                           std::to_string(leaf_points) + ",";
                csv.row(prefix + "scalar,min_ms," + std::to_string(scalar.min_ms) + ",ms");
                csv.row(prefix + "batched,min_ms," + std::to_string(batched.min_ms) + ",ms");
                csv.row(prefix + "ratio,scalar_over_batched," + std::to_string(ratio) + ",1");
            }
        }
        std::printf("\nвсяка редица е %s заявки, разпределени така, че общата работа да е една и съща\n",
                    "20 000 000 / точки в листа");
    }

    // -----------------------------------------------------------------
    // 6. Прореждане
    // -----------------------------------------------------------------
    section("6. Прореждане с вокселна решетка, 1000000 точки");
    header({{"ребро", 12}, {"минимум, ms", 14}, {"изходни точки", 14}, {"остават, %", 12}});
    {
        const PointCloud& cloud = clouds[3];
        for (const float leaf : {0.02F, 0.05F, 0.10F, 0.25F}) {
            VoxelGridStats stats;
            PointCloud reduced;
            const TimingStats time =
                measure(3, 1, [&] { reduced = voxel_downsample(cloud, leaf, &stats); });
            std::printf("%12.3f %14.2f %14zu %12.2f\n", static_cast<double>(leaf), time.min_ms,
                        stats.output_points, 100.0 * stats.reduction_ratio());
            csv.row("voxel,leaf" + std::to_string(leaf) + ",1000000,downsample,min_ms," +
                    std::to_string(time.min_ms) + ",ms");
        }
    }

    // -----------------------------------------------------------------
    // 7. Съвместяване: итерации спрямо шума
    // -----------------------------------------------------------------
    section("7. Съвместяване, итерации до сходимост спрямо шума");
    header({{"шум sigma", 10}, {"итерации", 12}, {"сходимост", 12}, {"отклонение", 14},
            {"ъгъл, градуси", 14}, {"време мин, ms", 14}});
    {
        SceneOptions scene_options;
        scene_options.plane_points = 30000;
        scene_options.sphere_points = 20000;
        scene_options.box_points = 20000;
        const PointCloud reference = make_scene(scene_options);

        RigidTransform truth;
        truth.rotation = rotation_axis_angle(0.31, -0.57, 0.76, 0.13);
        truth.translation = {0.180F, -0.120F, 0.075F};

        for (const float sigma : {0.000F, 0.002F, 0.005F, 0.010F, 0.020F, 0.040F}) {
            const PointCloud target = add_gaussian_noise(reference, sigma, 101U);
            const PointCloud source = transform_cloud(
                add_gaussian_noise(random_subset(reference, 0.75, 202U), sigma, 303U),
                truth.inverse());

            const PointCloud target_small = voxel_downsample(target, 0.05F);
            const PointCloud source_small = voxel_downsample(source, 0.05F);
            const KdTree index(target_small, KdTreeOptions{32});

            IcpOptions options;
            options.max_iterations = 60;
            options.max_correspondence_distance = 0.5F;

            IcpResult result;
            const TimingStats time = measure(3, 1, [&] {
                result = icp_point_to_point(source_small, target_small, index, options);
            });

            const Mat3 relative = result.transform.rotation * truth.rotation.transposed();
            const double angle_deg = rotation_angle(relative) * 180.0 / 3.14159265358979;
            std::printf("%10.3f %12d %12s %14.6f %14.4f %12.2f\n", static_cast<double>(sigma),
                        result.iterations, result.converged ? "да" : "не", result.rmse, angle_deg,
                        time.min_ms);

            const std::string prefix = "icp,sigma" + std::to_string(sigma) + "," +
                                       std::to_string(source_small.size()) + ",";
            csv.row(prefix + "iterations,count," + std::to_string(result.iterations) + ",1");
            csv.row(prefix + "rmse,value," + std::to_string(result.rmse) + ",units");
            csv.row(prefix + "angle_error,deg," + std::to_string(angle_deg) + ",deg");
            csv.row(prefix + "time,min_ms," + std::to_string(time.min_ms) + ",ms");
        }
    }

    // -----------------------------------------------------------------
    // 8. Растерният конвейер
    // -----------------------------------------------------------------
    section("8. Растерен конвейер, 1024 x 768");
    header({{"стъпка", 28}, {"минимум, ms", 14}, {"медиана, ms", 14}});
    {
        ImageSceneOptions image_options;
        image_options.width = 1024;
        image_options.height = 768;
        const Image8 scene = make_test_image(image_options);
        const ImageF grey = to_float(scene);

        ImageF blurred;
        const TimingStats blur_time = measure(5, 1, [&] { blurred = gaussian_blur(grey, 1.6F); });
        Gradient gradient;
        const TimingStats sobel_time = measure(5, 1, [&] { gradient = sobel(blurred); });
        ImageF thin;
        const TimingStats nms_time = measure(5, 1, [&] { thin = non_maximum_suppression(gradient); });
        Image8 edges;
        const TimingStats hyst_time =
            measure(5, 1, [&] { edges = hysteresis_threshold(thin, 40.0F, 110.0F); });
        const Image8 mask = threshold(blurred, 110.0F);
        LabelResult labels;
        const TimingStats label_time =
            measure(5, 1, [&] { labels = connected_components(mask, Connectivity::Eight, 50); });

        const struct {
            const char* name;
            const TimingStats* stats;
        } rows[] = {{"гаусово изглаждане", &blur_time},
                    {"градиент по Собел", &sobel_time},
                    {"потискане на немаксимуми", &nms_time},
                    {"двупрагово решение", &hyst_time},
                    {"свързани компоненти", &label_time}};
        for (const auto& row : rows) {
            std::printf("%s %14.3f %14.3f\n", pad_left(row.name, 28).c_str(), row.stats->min_ms,
                        row.stats->median_ms);
            csv.row(std::string("image,") + row.name + ",786432,pipeline,min_ms," +
                    std::to_string(row.stats->min_ms) + ",ms");
        }
        std::printf("\nнамерени области: %zu\n", labels.components.size());
    }

    std::printf("\nготово. CSV: %s\n", csv_path.c_str());
    return 0;
}
