#include <cmath>

#include "pointforge/transform.hpp"
#include "test_framework.hpp"

using namespace pointforge;

namespace {

double frobenius_difference(const Mat3& a, const Mat3& b) {
    double sum = 0.0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            const double d = a(i, j) - b(i, j);
            sum += d * d;
        }
    }
    return std::sqrt(sum);
}

}  // namespace

PF_TEST(geometry, svd_reconstructs_the_matrix) {
    Mat3 a;
    a(0, 0) = 1.5;
    a(0, 1) = -0.4;
    a(0, 2) = 2.1;
    a(1, 0) = 0.3;
    a(1, 1) = 3.2;
    a(1, 2) = -1.1;
    a(2, 0) = -2.0;
    a(2, 1) = 0.7;
    a(2, 2) = 0.9;

    const Svd3 svd = svd3(a);

    Mat3 sigma = Mat3::zero();
    for (int i = 0; i < 3; ++i) sigma(i, i) = svd.s[static_cast<std::size_t>(i)];
    const Mat3 reconstructed = svd.u * sigma * svd.v.transposed();

    PF_CHECK_NEAR(frobenius_difference(a, reconstructed), 0.0, 1e-10);
}

PF_TEST(geometry, svd_factors_are_orthogonal) {
    Mat3 a;
    a(0, 0) = 4.0;
    a(0, 1) = 1.0;
    a(0, 2) = 0.0;
    a(1, 0) = -1.0;
    a(1, 1) = 2.5;
    a(1, 2) = 3.0;
    a(2, 0) = 0.5;
    a(2, 1) = -2.0;
    a(2, 2) = 1.0;

    const Svd3 svd = svd3(a);
    PF_CHECK_NEAR(frobenius_difference(svd.u * svd.u.transposed(), Mat3::identity()), 0.0, 1e-10);
    PF_CHECK_NEAR(frobenius_difference(svd.v * svd.v.transposed(), Mat3::identity()), 0.0, 1e-10);
    // Сингулярните числа са неотрицателни по определение.
    for (int i = 0; i < 3; ++i) PF_CHECK(svd.s[static_cast<std::size_t>(i)] >= 0.0);
}

PF_TEST(geometry, svd_survives_a_rank_deficient_matrix) {
    // Втори ред, равен на първия: рангът е 2 и едно сингулярно число е нула.
    // Случаят се среща при облак, легнал точно върху равнина.
    Mat3 a = Mat3::zero();
    a(0, 0) = 1.0;
    a(0, 1) = 2.0;
    a(1, 0) = 1.0;
    a(1, 1) = 2.0;
    a(2, 2) = 3.0;

    const Svd3 svd = svd3(a);
    Mat3 sigma = Mat3::zero();
    for (int i = 0; i < 3; ++i) sigma(i, i) = svd.s[static_cast<std::size_t>(i)];
    PF_CHECK_NEAR(frobenius_difference(a, svd.u * sigma * svd.v.transposed()), 0.0, 1e-9);
}

PF_TEST(geometry, axis_angle_rotation_is_a_rotation) {
    const Mat3 r = rotation_axis_angle(0.3, -0.5, 0.81, 0.7);
    PF_CHECK_NEAR(frobenius_difference(r * r.transposed(), Mat3::identity()), 0.0, 1e-12);
    PF_CHECK_NEAR(r.determinant(), 1.0, 1e-12);
    PF_CHECK_NEAR(rotation_angle(r), 0.7, 1e-12);
}

PF_TEST(geometry, rotation_around_z_moves_x_to_y) {
    const Mat3 r = rotation_axis_angle(0.0, 0.0, 1.0, 3.14159265358979 / 2.0);
    const Point3 p = r * Point3{1.0F, 0.0F, 0.0F};
    PF_CHECK_NEAR(p.x, 0.0, 1e-6);
    PF_CHECK_NEAR(p.y, 1.0, 1e-6);
    PF_CHECK_NEAR(p.z, 0.0, 1e-6);
}

PF_TEST(geometry, inverse_undoes_the_transform) {
    RigidTransform t;
    t.rotation = rotation_axis_angle(1.0, 0.4, -0.2, 0.9);
    t.translation = {2.5F, -1.0F, 0.75F};

    const Point3 p{0.3F, -1.7F, 4.2F};
    const Point3 round_trip = t.inverse().apply(t.apply(p));
    PF_CHECK_NEAR(round_trip.x, p.x, 1e-5);
    PF_CHECK_NEAR(round_trip.y, p.y, 1e-5);
    PF_CHECK_NEAR(round_trip.z, p.z, 1e-5);
}

PF_TEST(geometry, compose_applies_the_argument_first) {
    RigidTransform a;
    a.rotation = rotation_axis_angle(0.0, 0.0, 1.0, 0.5);
    a.translation = {1.0F, 0.0F, 0.0F};

    RigidTransform b;
    b.rotation = rotation_axis_angle(0.0, 1.0, 0.0, -0.3);
    b.translation = {0.0F, 2.0F, -1.0F};

    const Point3 p{0.5F, 1.5F, -2.0F};
    const Point3 composed = a.compose(b).apply(p);
    const Point3 sequential = a.apply(b.apply(p));
    PF_CHECK_NEAR(composed.x, sequential.x, 1e-5);
    PF_CHECK_NEAR(composed.y, sequential.y, 1e-5);
    PF_CHECK_NEAR(composed.z, sequential.z, 1e-5);
}

PF_TEST(geometry, transform_cloud_matches_pointwise_application) {
    PointCloud cloud;
    for (int i = 0; i < 25; ++i) {
        cloud.push_back(static_cast<float>(i) * 0.3F, static_cast<float>(i % 7) - 3.0F,
                        static_cast<float>(i % 5) * -0.8F);
    }

    RigidTransform t;
    t.rotation = rotation_axis_angle(0.2, 0.9, 0.1, 1.2);
    t.translation = {-3.0F, 0.5F, 2.25F};

    const PointCloud moved = transform_cloud(cloud, t);
    PF_CHECK_EQ(moved.size(), cloud.size());
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        const Point3 expected = t.apply(cloud.point(i));
        PF_CHECK_NEAR(moved.point(i).x, expected.x, 1e-5);
        PF_CHECK_NEAR(moved.point(i).y, expected.y, 1e-5);
        PF_CHECK_NEAR(moved.point(i).z, expected.z, 1e-5);
    }
}

PF_TEST(geometry, bounds_and_centroid_of_a_known_cloud) {
    PointCloud cloud;
    cloud.push_back(-1.0F, 0.0F, 5.0F);
    cloud.push_back(3.0F, 2.0F, -5.0F);
    cloud.push_back(1.0F, -2.0F, 0.0F);

    const Aabb box = cloud.bounds();
    PF_CHECK_NEAR(box.min.x, -1.0, 1e-6);
    PF_CHECK_NEAR(box.max.x, 3.0, 1e-6);
    PF_CHECK_NEAR(box.min.z, -5.0, 1e-6);
    PF_CHECK_NEAR(box.max.z, 5.0, 1e-6);
    // Разтегът по z е 10, по x е 4, по y е 4.
    PF_CHECK_EQ(box.widest_axis(), 2);

    const Point3 c = cloud.centroid();
    PF_CHECK_NEAR(c.x, 1.0, 1e-6);
    PF_CHECK_NEAR(c.y, 0.0, 1e-6);
    PF_CHECK_NEAR(c.z, 0.0, 1e-6);
}
