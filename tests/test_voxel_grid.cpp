#include <cmath>
#include <set>

#include "pointforge/synthetic.hpp"
#include "pointforge/voxel_grid.hpp"
#include "test_framework.hpp"

using namespace pointforge;

PF_TEST(voxel_grid, downsampling_reduces_the_cloud) {
    const PointCloud cloud = make_scene(SceneOptions{5000, 3000, 3000});
    VoxelGridStats stats;
    const PointCloud reduced = voxel_downsample(cloud, 0.25F, &stats);

    PF_CHECK(reduced.size() < cloud.size());
    PF_CHECK(!reduced.empty());
    PF_CHECK_EQ(stats.input_points, cloud.size());
    PF_CHECK_EQ(stats.output_points, reduced.size());
    PF_CHECK_NEAR(stats.leaf_size, 0.25, 1e-9);
    PF_CHECK(stats.reduction_ratio() > 0.0 && stats.reduction_ratio() < 1.0);
}

PF_TEST(voxel_grid, a_larger_leaf_gives_fewer_points) {
    const PointCloud cloud = make_scene(SceneOptions{4000, 2000, 2000});
    const std::size_t fine = voxel_downsample(cloud, 0.1F).size();
    const std::size_t medium = voxel_downsample(cloud, 0.4F).size();
    const std::size_t coarse = voxel_downsample(cloud, 1.6F).size();
    PF_CHECK(fine > medium);
    PF_CHECK(medium > coarse);
}

PF_TEST(voxel_grid, output_points_stay_inside_their_cell) {
    const PointCloud cloud = make_scene(SceneOptions{2000, 1000, 1000});
    const float leaf = 0.3F;
    const PointCloud reduced = voxel_downsample(cloud, leaf);
    const Aabb box = cloud.bounds();

    // Центърът на тежестта на кубче лежи вътре в него, значи и вътре в
    // ограждащия паралелепипед на входа.
    for (std::size_t i = 0; i < reduced.size(); ++i) {
        const Point3 p = reduced.point(i);
        PF_CHECK(p.x >= box.min.x - 1e-4F && p.x <= box.max.x + 1e-4F);
        PF_CHECK(p.y >= box.min.y - 1e-4F && p.y <= box.max.y + 1e-4F);
        PF_CHECK(p.z >= box.min.z - 1e-4F && p.z <= box.max.z + 1e-4F);
    }
}

PF_TEST(voxel_grid, a_leaf_finer_than_the_spacing_keeps_every_point) {
    // Решетка със стъпка 1.0. Ребро 0.1 слага всяка точка в отделно кубче.
    PointCloud cloud;
    for (int x = 0; x < 6; ++x) {
        for (int y = 0; y < 6; ++y) {
            cloud.push_back(static_cast<float>(x), static_cast<float>(y), 0.0F);
        }
    }
    const PointCloud reduced = voxel_downsample(cloud, 0.1F);
    PF_CHECK_EQ(reduced.size(), cloud.size());
}

PF_TEST(voxel_grid, a_leaf_over_the_whole_cloud_gives_the_centroid) {
    PointCloud cloud;
    cloud.push_back(0.0F, 0.0F, 0.0F);
    cloud.push_back(1.0F, 0.0F, 0.0F);
    cloud.push_back(0.0F, 1.0F, 0.0F);
    cloud.push_back(1.0F, 1.0F, 2.0F);

    const PointCloud reduced = voxel_downsample(cloud, 100.0F);
    PF_CHECK_EQ(reduced.size(), std::size_t{1});
    const Point3 c = cloud.centroid();
    PF_CHECK_NEAR(reduced.point(0).x, c.x, 1e-5);
    PF_CHECK_NEAR(reduced.point(0).y, c.y, 1e-5);
    PF_CHECK_NEAR(reduced.point(0).z, c.z, 1e-5);
}

PF_TEST(voxel_grid, the_result_is_repeatable) {
    const PointCloud cloud = make_scene(SceneOptions{1500, 800, 800});
    const PointCloud first = voxel_downsample(cloud, 0.35F);
    const PointCloud second = voxel_downsample(cloud, 0.35F);
    PF_CHECK_EQ(first.size(), second.size());
    for (std::size_t i = 0; i < first.size(); ++i) {
        PF_CHECK_NEAR(first.point(i).x, second.point(i).x, 0.0);
        PF_CHECK_NEAR(first.point(i).y, second.point(i).y, 0.0);
        PF_CHECK_NEAR(first.point(i).z, second.point(i).z, 0.0);
    }
}

PF_TEST(voxel_grid, invalid_leaf_sizes_are_rejected) {
    const PointCloud cloud = make_scene(SceneOptions{100, 50, 50});
    PF_CHECK_THROWS(voxel_downsample(cloud, 0.0F));
    PF_CHECK_THROWS(voxel_downsample(cloud, -1.0F));
    // Ребро, което дава над 2^21 кубчета по ос, се отхвърля, вместо ключът да
    // препълни мълчаливо.
    PF_CHECK_THROWS(voxel_downsample(cloud, 1e-7F));

    // Празен облак не е грешка.
    const PointCloud empty;
    PF_CHECK(voxel_downsample(empty, 1.0F).empty());
}

PF_TEST(voxel_grid, the_grid_shape_matches_the_bounds) {
    PointCloud cloud;
    cloud.push_back(0.0F, 0.0F, 0.0F);
    cloud.push_back(9.5F, 4.5F, 0.0F);

    const VoxelGridShape shape = voxel_grid_shape(cloud, 1.0F);
    PF_CHECK_EQ(shape.nx, std::uint64_t{10});
    PF_CHECK_EQ(shape.ny, std::uint64_t{5});
    PF_CHECK_EQ(shape.nz, std::uint64_t{1});
}
