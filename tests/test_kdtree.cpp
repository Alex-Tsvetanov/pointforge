#include <algorithm>
#include <cmath>
#include <random>
#include <set>
#include <vector>

#include "pointforge/kdtree.hpp"
#include "pointforge/synthetic.hpp"
#include "test_framework.hpp"

using namespace pointforge;

namespace {

std::vector<std::uint32_t> indices_of(const std::vector<Neighbor>& neighbors) {
    std::vector<std::uint32_t> out;
    out.reserve(neighbors.size());
    for (const Neighbor& n : neighbors) out.push_back(n.index);
    std::sort(out.begin(), out.end());
    return out;
}

PointCloud random_cloud(std::size_t count, std::uint32_t seed, float spread = 10.0F) {
    std::mt19937 rng(seed);
    PointCloud cloud;
    cloud.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        const auto draw = [&]() {
            return (static_cast<float>(std::generate_canonical<double, 32>(rng)) - 0.5F) * spread;
        };
        cloud.push_back(draw(), draw(), draw());
    }
    return cloud;
}

// Сравнението е по МНОЖЕСТВОТО от разстояния, а не по индексите: при равни
// разстояния кой от двата еднакво отдалечени съседа ще влезе не е част от
// договора и еталонът има същото право на избор като индекса.
void check_same_distances(const std::vector<Neighbor>& a, const std::vector<Neighbor>& b,
                          float tolerance) {
    PF_CHECK_EQ(a.size(), b.size());
    if (a.size() != b.size()) return;
    std::vector<float> da;
    std::vector<float> db;
    for (const Neighbor& n : a) da.push_back(n.squared_distance);
    for (const Neighbor& n : b) db.push_back(n.squared_distance);
    std::sort(da.begin(), da.end());
    std::sort(db.begin(), db.end());
    for (std::size_t i = 0; i < da.size(); ++i) PF_CHECK_NEAR(da[i], db[i], tolerance);
}

}  // namespace

PF_TEST(kdtree, knn_agrees_with_exhaustive_search) {
    const PointCloud cloud = random_cloud(2000, 7U);
    const KdTree tree(cloud);
    const PointCloud queries = random_cloud(40, 99U, 12.0F);

    for (const std::size_t k : {std::size_t{1}, std::size_t{5}, std::size_t{32}}) {
        for (std::size_t q = 0; q < queries.size(); ++q) {
            const Point3 query = queries.point(q);
            const std::vector<Neighbor> expected = brute_force_knn(cloud, query, k);
            const std::vector<Neighbor> got = tree.knn(query, k);
            check_same_distances(expected, got, 1e-5F);
            // Резултатът трябва да е подреден по нарастващо разстояние.
            for (std::size_t i = 1; i < got.size(); ++i) {
                PF_CHECK(got[i - 1].squared_distance <= got[i].squared_distance);
            }
        }
    }
}

PF_TEST(kdtree, scalar_batched_and_simd_paths_agree) {
    const PointCloud cloud = random_cloud(3000, 13U);
    const KdTree tree(cloud, KdTreeOptions{40});
    const PointCloud queries = random_cloud(60, 21U, 11.0F);

    for (std::size_t q = 0; q < queries.size(); ++q) {
        const Point3 query = queries.point(q);

        const std::vector<Neighbor> scalar = tree.knn(query, 16, NnPath::Scalar);
        const std::vector<Neighbor> batched = tree.knn(query, 16, NnPath::Batched);
        const std::vector<Neighbor> simd = tree.knn(query, 16, NnPath::Simd);
        check_same_distances(scalar, batched, 0.0F);
        check_same_distances(scalar, simd, 0.0F);

        const std::vector<Neighbor> radius_scalar = tree.radius_search(query, 2.0F, NnPath::Scalar);
        const std::vector<Neighbor> radius_batched = tree.radius_search(query, 2.0F, NnPath::Batched);
        const std::vector<Neighbor> radius_simd = tree.radius_search(query, 2.0F, NnPath::Simd);
        PF_CHECK_EQ(indices_of(radius_scalar), indices_of(radius_batched));
        PF_CHECK_EQ(indices_of(radius_scalar), indices_of(radius_simd));

        Neighbor near_scalar;
        Neighbor near_batched;
        Neighbor near_simd;
        PF_CHECK(tree.nearest(query, near_scalar, NnPath::Scalar));
        PF_CHECK(tree.nearest(query, near_batched, NnPath::Batched));
        PF_CHECK(tree.nearest(query, near_simd, NnPath::Simd));
        PF_CHECK_NEAR(near_scalar.squared_distance, near_batched.squared_distance, 0.0);
        PF_CHECK_NEAR(near_scalar.squared_distance, near_simd.squared_distance, 0.0);
    }
}

PF_TEST(kdtree, radius_search_agrees_with_exhaustive_search) {
    const PointCloud cloud = random_cloud(1500, 31U);
    const KdTree tree(cloud);
    const PointCloud queries = random_cloud(30, 41U, 10.0F);

    for (const float radius : {0.4F, 1.5F, 4.0F}) {
        for (std::size_t q = 0; q < queries.size(); ++q) {
            const Point3 query = queries.point(q);
            PF_CHECK_EQ(indices_of(tree.radius_search(query, radius)),
                        indices_of(brute_force_radius(cloud, query, radius)));
        }
    }
}

PF_TEST(kdtree, nearest_matches_the_first_knn_result) {
    const PointCloud cloud = random_cloud(1200, 53U);
    const KdTree tree(cloud);
    const PointCloud queries = random_cloud(80, 67U, 12.0F);

    for (std::size_t q = 0; q < queries.size(); ++q) {
        const Point3 query = queries.point(q);
        Neighbor hit;
        PF_CHECK(tree.nearest(query, hit));
        const std::vector<Neighbor> expected = brute_force_knn(cloud, query, 1);
        PF_CHECK_NEAR(hit.squared_distance, expected.front().squared_distance, 1e-6);
    }
}

PF_TEST(kdtree, leaf_size_does_not_change_the_answer) {
    const PointCloud cloud = random_cloud(2500, 77U);
    const PointCloud queries = random_cloud(25, 78U, 10.0F);
    const KdTree reference(cloud, KdTreeOptions{1});

    for (const std::uint32_t leaf : {std::uint32_t{4}, std::uint32_t{16}, std::uint32_t{64},
                                     std::uint32_t{256}}) {
        const KdTree tree(cloud, KdTreeOptions{leaf});
        PF_CHECK_EQ(tree.leaf_size(), leaf);
        for (std::size_t q = 0; q < queries.size(); ++q) {
            check_same_distances(reference.knn(queries.point(q), 8), tree.knn(queries.point(q), 8),
                                 1e-6F);
        }
    }
}

PF_TEST(kdtree, degenerate_clouds_do_not_break_the_tree) {
    // Всички точки съвпадат: разделянето по медиана не напредва и построяването
    // трябва да спре с лист, а не да се върти.
    PointCloud identical;
    for (int i = 0; i < 300; ++i) identical.push_back(2.0F, -1.0F, 0.5F);
    const KdTree same(identical, KdTreeOptions{8});
    PF_CHECK_EQ(same.size(), std::size_t{300});
    const std::vector<Neighbor> found = same.knn(Point3{2.0F, -1.0F, 0.5F}, 5);
    PF_CHECK_EQ(found.size(), std::size_t{5});
    for (const Neighbor& n : found) PF_CHECK_NEAR(n.squared_distance, 0.0, 1e-9);

    // Всички точки на една права: две от трите оси имат нулев разтег.
    PointCloud collinear;
    for (int i = 0; i < 500; ++i) collinear.push_back(static_cast<float>(i) * 0.1F, 3.0F, -2.0F);
    const KdTree line(collinear, KdTreeOptions{16});
    const Point3 query{12.34F, 3.0F, -2.0F};
    check_same_distances(brute_force_knn(collinear, query, 7), line.knn(query, 7), 1e-6F);
}

PF_TEST(kdtree, boundary_cases_of_k_and_of_the_empty_tree) {
    const PointCloud cloud = random_cloud(50, 101U);
    const KdTree tree(cloud, KdTreeOptions{8});

    // k над броя на точките връща всички, не хвърля.
    PF_CHECK_EQ(tree.knn(Point3{}, 500).size(), std::size_t{50});
    PF_CHECK(tree.knn(Point3{}, 0).empty());

    const KdTree empty;
    PF_CHECK(empty.empty());
    PF_CHECK(empty.knn(Point3{}, 5).empty());
    PF_CHECK(empty.radius_search(Point3{}, 1.0F).empty());
    Neighbor hit;
    PF_CHECK(!empty.nearest(Point3{}, hit));

    // Отрицателен радиус е празен резултат, не грешка при обхождането.
    PF_CHECK(tree.radius_search(Point3{}, -1.0F).empty());
}

PF_TEST(kdtree, memory_report_grows_with_the_cloud) {
    const KdTree small(random_cloud(1000, 5U));
    const KdTree large(random_cloud(8000, 5U));
    PF_CHECK(small.memory_bytes() > 0);
    PF_CHECK(large.memory_bytes() > small.memory_bytes());
    // Долна граница: три координати и един индекс на точка.
    PF_CHECK(large.memory_bytes() >= 8000 * (3 * sizeof(float) + sizeof(std::uint32_t)));
    PF_CHECK(large.node_count() > 1);
}

PF_TEST(kdtree, queries_on_a_structured_scene_agree_with_the_oracle) {
    // Сцена с рязко различна плътност по частите си. Точно там равномерната
    // решетка би се държала зле, а k-d дървото трябва да остане точно.
    const PointCloud cloud = make_scene(SceneOptions{3000, 2000, 1500});
    const KdTree tree(cloud, KdTreeOptions{24});

    const Point3 queries[] = {{0.0F, 0.0F, 0.0F},   {-1.5F, 1.2F, 1.0F}, {1.6F, -1.0F, 0.8F},
                              {3.9F, 3.9F, 0.0F},   {0.0F, 0.0F, 10.0F}};
    for (const Point3& query : queries) {
        check_same_distances(brute_force_knn(cloud, query, 20), tree.knn(query, 20), 1e-5F);
        PF_CHECK_EQ(indices_of(tree.radius_search(query, 0.9F)),
                    indices_of(brute_force_radius(cloud, query, 0.9F)));
    }
}
