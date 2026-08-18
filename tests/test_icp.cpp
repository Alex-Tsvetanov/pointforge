#include <cmath>
#include <numeric>
#include <vector>

#include "pointforge/icp.hpp"
#include "pointforge/synthetic.hpp"
#include "test_framework.hpp"

using namespace pointforge;

namespace {

RigidTransform known_transform(double angle, float tx, float ty, float tz) {
    RigidTransform t;
    t.rotation = rotation_axis_angle(0.35, -0.62, 0.70, angle);
    t.translation = {tx, ty, tz};
    return t;
}

// Разлика между две корави преобразувания, изразена като ъгъл и като дължина.
struct TransformError {
    double angle_rad;
    double translation;
};

TransformError error_against(const RigidTransform& got, const RigidTransform& expected) {
    const Mat3 relative = got.rotation * expected.rotation.transposed();
    const Point3 d = got.translation - expected.translation;
    return {rotation_angle(relative),
            std::sqrt(static_cast<double>(d.x) * d.x + static_cast<double>(d.y) * d.y +
                      static_cast<double>(d.z) * d.z)};
}

std::vector<std::uint32_t> identity_indices(std::size_t n) {
    std::vector<std::uint32_t> indices(n);
    std::iota(indices.begin(), indices.end(), 0U);
    return indices;
}

}  // namespace

PF_TEST(icp, kabsch_recovers_an_exact_transform) {
    const PointCloud source = make_scene(SceneOptions{400, 300, 300});
    const RigidTransform truth = known_transform(0.7, 1.25F, -0.5F, 2.0F);
    const PointCloud target = transform_cloud(source, truth);

    const std::vector<std::uint32_t> indices = identity_indices(source.size());
    const RigidTransform estimated = estimate_rigid_transform(source, target, indices, indices);

    const TransformError error = error_against(estimated, truth);
    PF_CHECK_NEAR(error.angle_rad, 0.0, 1e-5);
    PF_CHECK_NEAR(error.translation, 0.0, 1e-4);
}

PF_TEST(icp, kabsch_returns_identity_for_too_few_pairs) {
    PointCloud a;
    a.push_back(0.0F, 0.0F, 0.0F);
    a.push_back(1.0F, 0.0F, 0.0F);
    PointCloud b;
    b.push_back(5.0F, 5.0F, 5.0F);
    b.push_back(6.0F, 5.0F, 5.0F);

    const std::vector<std::uint32_t> indices = identity_indices(2);
    const RigidTransform estimated = estimate_rigid_transform(a, b, indices, indices);
    PF_CHECK_NEAR(rotation_angle(estimated.rotation), 0.0, 1e-12);
    PF_CHECK_NEAR(estimated.translation.x, 0.0, 1e-12);
}

PF_TEST(icp, kabsch_never_returns_a_reflection) {
    // Точки върху равнина: кръстосаната ковариация е с ранг 2 и наивното
    // сглобяване от сингулярното разлагане може да даде отражение.
    const PointCloud source = make_plane(500, 3.0F, 0.0F, 4242U);
    const RigidTransform truth = known_transform(0.4, 0.5F, 0.25F, 0.0F);
    const PointCloud target = transform_cloud(source, truth);

    const std::vector<std::uint32_t> indices = identity_indices(source.size());
    const RigidTransform estimated = estimate_rigid_transform(source, target, indices, indices);
    PF_CHECK(estimated.rotation.determinant() > 0.99);
    PF_CHECK(estimated.rotation.determinant() < 1.01);
}

PF_TEST(icp, recovers_a_known_transform_without_noise) {
    const PointCloud target = make_scene(SceneOptions{3000, 2000, 2000});
    const RigidTransform truth = known_transform(0.20, 0.30F, -0.22F, 0.15F);
    // source лежи там, откъдето ICP тръгва: обратното на истината върху target.
    const PointCloud source = transform_cloud(target, truth.inverse());

    IcpOptions options;
    options.max_correspondence_distance = 1.0F;
    options.max_iterations = 60;
    const IcpResult result = icp_point_to_point(source, target, options);

    const TransformError error = error_against(result.transform, truth);
    PF_CHECK(result.converged);
    PF_CHECK(result.iterations > 0);
    PF_CHECK_NEAR(error.angle_rad, 0.0, 5e-3);
    PF_CHECK_NEAR(error.translation, 0.0, 2e-2);
    PF_CHECK(result.rmse < 0.01);
}

PF_TEST(icp, recovers_a_known_transform_with_noise) {
    const PointCloud clean = make_scene(SceneOptions{4000, 2500, 2500});
    const float sigma = 0.01F;
    const PointCloud target = add_gaussian_noise(clean, sigma, 11U);

    const RigidTransform truth = known_transform(0.16, -0.25F, 0.18F, 0.12F);
    // Различни точки в двата облака, а не едни и същи, преместени.
    const PointCloud source =
        transform_cloud(add_gaussian_noise(random_subset(clean, 0.7, 3U), sigma, 12U),
                        truth.inverse());

    IcpOptions options;
    options.max_correspondence_distance = 0.8F;
    options.max_iterations = 80;
    const IcpResult result = icp_point_to_point(source, target, options);

    const TransformError error = error_against(result.transform, truth);
    PF_CHECK_NEAR(error.angle_rad, 0.0, 2e-2);
    PF_CHECK_NEAR(error.translation, 0.0, 5e-2);
    // Отклонението не може да падне под шума, но не бива и да го надхвърля
    // многократно.
    PF_CHECK(result.rmse < 8.0 * sigma);
    PF_CHECK(result.correspondences > source.size() / 2);
}

PF_TEST(icp, an_already_aligned_pair_converges_at_once) {
    const PointCloud cloud = make_scene(SceneOptions{1500, 900, 900});
    IcpOptions options;
    options.max_correspondence_distance = 0.5F;
    const IcpResult result = icp_point_to_point(cloud, cloud, options);

    PF_CHECK(result.converged);
    PF_CHECK(result.iterations <= 2);
    PF_CHECK_NEAR(result.rmse, 0.0, 1e-6);
    PF_CHECK_NEAR(rotation_angle(result.transform.rotation), 0.0, 1e-6);
}

PF_TEST(icp, an_initial_guess_is_used) {
    const PointCloud target = make_scene(SceneOptions{2000, 1200, 1200});
    const RigidTransform truth = known_transform(0.55, 1.10F, -0.80F, 0.60F);
    const PointCloud source = transform_cloud(target, truth.inverse());

    IcpOptions options;
    options.max_correspondence_distance = 0.6F;
    options.max_iterations = 40;

    // Без начално предположение отместването е над прага за съответствие и
    // съвместяването не тръгва към истината.
    const IcpResult cold = icp_point_to_point(source, target, options);
    // С грубо начално предположение задачата става решима.
    RigidTransform guess = truth;
    guess.translation = {truth.translation.x * 0.8F, truth.translation.y * 0.8F,
                         truth.translation.z * 0.8F};
    const IcpResult warm = icp_point_to_point(source, target, options, guess);

    PF_CHECK(warm.rmse <= cold.rmse);
    PF_CHECK_NEAR(error_against(warm.transform, truth).angle_rad, 0.0, 2e-2);
}

PF_TEST(icp, both_query_paths_give_the_same_registration) {
    const PointCloud target = make_scene(SceneOptions{2000, 1200, 1200});
    const RigidTransform truth = known_transform(0.18, 0.22F, -0.16F, 0.10F);
    const PointCloud source = transform_cloud(target, truth.inverse());

    IcpOptions scalar_options;
    scalar_options.path = NnPath::Scalar;
    scalar_options.max_correspondence_distance = 0.8F;
    IcpOptions batched_options = scalar_options;
    batched_options.path = NnPath::Batched;

    const IcpResult scalar = icp_point_to_point(source, target, scalar_options);
    const IcpResult batched = icp_point_to_point(source, target, batched_options);

    PF_CHECK_EQ(scalar.iterations, batched.iterations);
    PF_CHECK_NEAR(scalar.rmse, batched.rmse, 1e-9);
    PF_CHECK_NEAR(error_against(scalar.transform, batched.transform).angle_rad, 0.0, 1e-9);
}

PF_TEST(icp, empty_input_returns_the_initial_guess) {
    const PointCloud empty;
    const PointCloud cloud = make_scene(SceneOptions{100, 50, 50});
    const RigidTransform guess = known_transform(0.3, 1.0F, 2.0F, 3.0F);

    const IcpResult from_empty_source = icp_point_to_point(empty, cloud, IcpOptions{}, guess);
    PF_CHECK_EQ(from_empty_source.iterations, 0);
    // Допускът е 1e-6, не 1e-12: ъгълът се получава през acos от следата, а
    // acos около нула умножава грешката по 1/sqrt, тоест около 1e-8 от
    // машинната точност на double.
    PF_CHECK_NEAR(error_against(from_empty_source.transform, guess).angle_rad, 0.0, 1e-6);

    const IcpResult to_empty_target = icp_point_to_point(cloud, empty, IcpOptions{}, guess);
    PF_CHECK_EQ(to_empty_target.iterations, 0);
}

PF_TEST(icp, a_prebuilt_index_gives_the_same_answer) {
    const PointCloud target = make_scene(SceneOptions{1500, 900, 900});
    const RigidTransform truth = known_transform(0.12, 0.15F, -0.10F, 0.08F);
    const PointCloud source = transform_cloud(target, truth.inverse());

    IcpOptions options;
    options.max_correspondence_distance = 0.7F;

    const KdTree index(target);
    const IcpResult shared = icp_point_to_point(source, target, index, options);
    const IcpResult owned = icp_point_to_point(source, target, options);

    PF_CHECK_EQ(shared.iterations, owned.iterations);
    PF_CHECK_NEAR(shared.rmse, owned.rmse, 1e-12);
}
