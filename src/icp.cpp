#include "pointforge/icp.hpp"

#include <cmath>

namespace pointforge {
namespace {

struct Correspondences {
    std::vector<std::uint32_t> source_indices;
    std::vector<std::uint32_t> target_indices;
    double sum_squared = 0.0;

    void clear() {
        source_indices.clear();
        target_indices.clear();
        sum_squared = 0.0;
    }

    std::size_t size() const { return source_indices.size(); }

    double rmse() const {
        return size() == 0 ? 0.0 : std::sqrt(sum_squared / static_cast<double>(size()));
    }
};

void find_correspondences(const PointCloud& moved, const KdTree& target_index, float max_distance,
                          NnPath path, Correspondences& out) {
    out.clear();
    const float max_distance2 = max_distance * max_distance;
    Neighbor hit;
    for (std::size_t i = 0; i < moved.size(); ++i) {
        if (!target_index.nearest(moved.point(i), hit, path)) break;
        if (hit.squared_distance > max_distance2) continue;
        out.source_indices.push_back(static_cast<std::uint32_t>(i));
        out.target_indices.push_back(hit.index);
        out.sum_squared += static_cast<double>(hit.squared_distance);
    }
}

}  // namespace

// LISTING_BEGIN estimate_rigid_transform
RigidTransform estimate_rigid_transform(const PointCloud& source, const PointCloud& target,
                                        const std::vector<std::uint32_t>& source_indices,
                                        const std::vector<std::uint32_t>& target_indices) {
    RigidTransform result = RigidTransform::identity();
    const std::size_t n = std::min(source_indices.size(), target_indices.size());
    if (n < 3) return result;

    // Центровете се смятат в double. При облак, отместен далече от началото,
    // сумирането във float губи точно тези значещи цифри, които носят формата.
    double scx = 0.0;
    double scy = 0.0;
    double scz = 0.0;
    double tcx = 0.0;
    double tcy = 0.0;
    double tcz = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const Point3 s = source.point(source_indices[i]);
        const Point3 t = target.point(target_indices[i]);
        scx += s.x;
        scy += s.y;
        scz += s.z;
        tcx += t.x;
        tcy += t.y;
        tcz += t.z;
    }
    const double inv_n = 1.0 / static_cast<double>(n);
    scx *= inv_n;
    scy *= inv_n;
    scz *= inv_n;
    tcx *= inv_n;
    tcy *= inv_n;
    tcz *= inv_n;

    // Кръстосана ковариация H = sum (s - sc) * (t - tc)^T.
    Mat3 h = Mat3::zero();
    for (std::size_t i = 0; i < n; ++i) {
        const Point3 s = source.point(source_indices[i]);
        const Point3 t = target.point(target_indices[i]);
        const double sx = s.x - scx;
        const double sy = s.y - scy;
        const double sz = s.z - scz;
        const double tx = t.x - tcx;
        const double ty = t.y - tcy;
        const double tz = t.z - tcz;
        h(0, 0) += sx * tx;
        h(0, 1) += sx * ty;
        h(0, 2) += sx * tz;
        h(1, 0) += sy * tx;
        h(1, 1) += sy * ty;
        h(1, 2) += sy * tz;
        h(2, 0) += sz * tx;
        h(2, 1) += sz * ty;
        h(2, 2) += sz * tz;
    }

    Svd3 svd = svd3(h);
    Mat3 rotation = svd.v * svd.u.transposed();

    // Отражение вместо ротация. Получава се при шум върху почти симетричен
    // облак и се поправя чрез смяна на знака на стълба, съответстващ на
    // най-малкото сингулярно число.
    if (rotation.determinant() < 0.0) {
        int smallest = 0;
        for (int j = 1; j < 3; ++j) {
            if (svd.s[static_cast<std::size_t>(j)] < svd.s[static_cast<std::size_t>(smallest)]) {
                smallest = j;
            }
        }
        for (int i = 0; i < 3; ++i) svd.v(i, smallest) = -svd.v(i, smallest);
        rotation = svd.v * svd.u.transposed();
    }

    const Point3 rotated_center = rotation * Point3{static_cast<float>(scx), static_cast<float>(scy),
                                                    static_cast<float>(scz)};
    result.rotation = rotation;
    result.translation = {static_cast<float>(tcx) - rotated_center.x,
                          static_cast<float>(tcy) - rotated_center.y,
                          static_cast<float>(tcz) - rotated_center.z};
    return result;
}
// LISTING_END estimate_rigid_transform

IcpResult icp_point_to_point(const PointCloud& source, const PointCloud& target,
                             const KdTree& target_index, const IcpOptions& options,
                             const RigidTransform& initial_guess) {
    IcpResult result;
    result.transform = initial_guess;
    if (source.empty() || target.empty()) return result;

    PointCloud moved = transform_cloud(source, result.transform);
    Correspondences pairs;
    double previous_rmse = 0.0;
    bool have_previous = false;

    for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
        find_correspondences(moved, target_index, options.max_correspondence_distance, options.path,
                             pairs);
        result.correspondences = pairs.size();
        if (pairs.size() < 3) break;

        const double current_rmse = pairs.rmse();

        // Оценяването върви върху ВЕЧЕ преместения облак, затова получената
        // стъпка е добавка отляво към натрупаното преобразувание.
        const RigidTransform step =
            estimate_rigid_transform(moved, target, pairs.source_indices, pairs.target_indices);

        result.transform = step.compose(result.transform);
        moved = transform_cloud(source, result.transform);
        result.iterations = iteration + 1;

        const double angle = rotation_angle(step.rotation);
        const double shift = std::sqrt(static_cast<double>(step.translation.x) * step.translation.x +
                                       static_cast<double>(step.translation.y) * step.translation.y +
                                       static_cast<double>(step.translation.z) * step.translation.z);
        if (angle < options.rotation_epsilon && shift < options.translation_epsilon) {
            result.converged = true;
            break;
        }
        if (have_previous && std::fabs(previous_rmse - current_rmse) < options.rmse_epsilon) {
            result.converged = true;
            break;
        }
        previous_rmse = current_rmse;
        have_previous = true;
    }

    // Отчетеното отклонение е това на КРАЙНОТО положение, а не на последната
    // итерация преди обновяването. Разликата е цяла стъпка и точно тя е
    // печалбата от последната итерация.
    find_correspondences(moved, target_index, options.max_correspondence_distance, options.path, pairs);
    result.correspondences = pairs.size();
    result.rmse = pairs.rmse();
    return result;
}

IcpResult icp_point_to_point(const PointCloud& source, const PointCloud& target,
                             const IcpOptions& options, const RigidTransform& initial_guess) {
    KdTree index(target);
    return icp_point_to_point(source, target, index, options, initial_guess);
}

}  // namespace pointforge
