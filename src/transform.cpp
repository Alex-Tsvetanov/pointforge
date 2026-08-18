#include "pointforge/transform.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace pointforge {

Mat3 rotation_axis_angle(double ax, double ay, double az, double angle_rad) {
    const double norm = std::sqrt(ax * ax + ay * ay + az * az);
    if (norm <= 0.0) return Mat3::identity();
    ax /= norm;
    ay /= norm;
    az /= norm;
    const double c = std::cos(angle_rad);
    const double s = std::sin(angle_rad);
    const double t = 1.0 - c;

    Mat3 r;
    r(0, 0) = t * ax * ax + c;
    r(0, 1) = t * ax * ay - s * az;
    r(0, 2) = t * ax * az + s * ay;
    r(1, 0) = t * ax * ay + s * az;
    r(1, 1) = t * ay * ay + c;
    r(1, 2) = t * ay * az - s * ax;
    r(2, 0) = t * ax * az - s * ay;
    r(2, 1) = t * ay * az + s * ax;
    r(2, 2) = t * az * az + c;
    return r;
}

// Едностранен Якоби: върти двойки стълбове на копие на A, докато станат
// взаимно ортогонални. Тогава колонните норми са сингулярните числа, а
// нормираните стълбове са U. Натрупаните ротации дават V.
//
// Методът е предпочетен пред разлагане на A^T A, защото последното повдига
// числото на обусловеност на квадрат. Тук A е кръстосаната ковариация между
// два облака и при почти изроден облак, например точки върху равнина, тя вече
// е зле обусловена.
Svd3 svd3(const Mat3& a) {
    Mat3 work = a;
    Mat3 v = Mat3::identity();

    constexpr int kMaxSweeps = 32;
    constexpr double kTolerance = 1e-15;

    for (int sweep = 0; sweep < kMaxSweeps; ++sweep) {
        double off_diagonal = 0.0;
        for (int p = 0; p < 2; ++p) {
            for (int q = p + 1; q < 3; ++q) {
                double alpha = 0.0;
                double beta = 0.0;
                double gamma = 0.0;
                for (int i = 0; i < 3; ++i) {
                    alpha += work(i, p) * work(i, p);
                    beta += work(i, q) * work(i, q);
                    gamma += work(i, p) * work(i, q);
                }
                off_diagonal += std::fabs(gamma);
                if (gamma == 0.0 || std::fabs(gamma) <= kTolerance * std::sqrt(alpha * beta)) continue;

                const double zeta = (beta - alpha) / (2.0 * gamma);
                const double sign = (zeta >= 0.0) ? 1.0 : -1.0;
                const double t = sign / (std::fabs(zeta) + std::sqrt(1.0 + zeta * zeta));
                const double c = 1.0 / std::sqrt(1.0 + t * t);
                const double s = c * t;

                for (int i = 0; i < 3; ++i) {
                    const double wp = work(i, p);
                    const double wq = work(i, q);
                    work(i, p) = c * wp - s * wq;
                    work(i, q) = s * wp + c * wq;

                    const double vp = v(i, p);
                    const double vq = v(i, q);
                    v(i, p) = c * vp - s * vq;
                    v(i, q) = s * vp + c * vq;
                }
            }
        }
        if (off_diagonal <= kTolerance) break;
    }

    Svd3 result;
    result.v = v;
    result.u = Mat3::identity();

    std::array<double, 3> norms{{0.0, 0.0, 0.0}};
    double largest = 0.0;
    for (int j = 0; j < 3; ++j) {
        double norm = 0.0;
        for (int i = 0; i < 3; ++i) norm += work(i, j) * work(i, j);
        norms[static_cast<std::size_t>(j)] = std::sqrt(norm);
        largest = std::max(largest, norms[static_cast<std::size_t>(j)]);
    }

    // Праг за „нулев стълб“, отнесен към най-голямото сингулярно число. При
    // матрица с ранг под 3 съответният стълб на U е неопределен от A и трябва
    // да бъде ДОПЪЛНЕН до ортонормиран базис. Оставянето на единичен вектор на
    // негово място беше грешката, която правеше U неортогонална, а оттам и
    // произведението V * U^T понякога отражение вместо ротация.
    const double zero_threshold = std::max(1e-300, 1e-12 * largest);

    std::vector<int> degenerate;
    for (int j = 0; j < 3; ++j) {
        const double norm = norms[static_cast<std::size_t>(j)];
        result.s[static_cast<std::size_t>(j)] = norm;
        if (norm > zero_threshold) {
            for (int i = 0; i < 3; ++i) result.u(i, j) = work(i, j) / norm;
        } else {
            result.s[static_cast<std::size_t>(j)] = 0.0;
            degenerate.push_back(j);
        }
    }

    if (!degenerate.empty() && degenerate.size() < 3) {
        const auto column = [&](int j) {
            return std::array<double, 3>{{result.u(0, j), result.u(1, j), result.u(2, j)}};
        };
        const auto set_column = [&](int j, const std::array<double, 3>& c) {
            double norm = std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
            if (norm <= 0.0) norm = 1.0;
            for (int i = 0; i < 3; ++i) result.u(i, j) = c[static_cast<std::size_t>(i)] / norm;
        };
        const auto cross = [](const std::array<double, 3>& a, const std::array<double, 3>& b) {
            return std::array<double, 3>{{a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
                                          a[0] * b[1] - a[1] * b[0]}};
        };

        if (degenerate.size() == 1) {
            const int j = degenerate[0];
            const int a = (j + 1) % 3;
            const int b = (j + 2) % 3;
            // Векторното произведение на другите два стълба е единственият
            // (до знак) вектор, който допълва базиса.
            set_column(j, cross(column(a), column(b)));
        } else {
            // Ранг 1: единият стълб е известен, другите два се строят от него.
            const int good = 3 - degenerate[0] - degenerate[1];
            const std::array<double, 3> g = column(good);
            // Ос, която не е успоредна на известния стълб.
            std::array<double, 3> axis{{1.0, 0.0, 0.0}};
            if (std::fabs(g[0]) > 0.9) axis = {{0.0, 1.0, 0.0}};
            const std::array<double, 3> first = cross(g, axis);
            set_column(degenerate[0], first);
            set_column(degenerate[1], cross(g, column(degenerate[0])));
        }
    }
    return result;
}

PointCloud transform_cloud(const PointCloud& cloud, const RigidTransform& t) {
    PointCloud out;
    out.resize(cloud.size());
    const float* sx = cloud.xs();
    const float* sy = cloud.ys();
    const float* sz = cloud.zs();
    float* dx = out.xs();
    float* dy = out.ys();
    float* dz = out.zs();

    const double r00 = t.rotation(0, 0);
    const double r01 = t.rotation(0, 1);
    const double r02 = t.rotation(0, 2);
    const double r10 = t.rotation(1, 0);
    const double r11 = t.rotation(1, 1);
    const double r12 = t.rotation(1, 2);
    const double r20 = t.rotation(2, 0);
    const double r21 = t.rotation(2, 1);
    const double r22 = t.rotation(2, 2);

    for (std::size_t i = 0; i < cloud.size(); ++i) {
        const double px = sx[i];
        const double py = sy[i];
        const double pz = sz[i];
        dx[i] = static_cast<float>(r00 * px + r01 * py + r02 * pz + t.translation.x);
        dy[i] = static_cast<float>(r10 * px + r11 * py + r12 * pz + t.translation.y);
        dz[i] = static_cast<float>(r20 * px + r21 * py + r22 * pz + t.translation.z);
    }
    return out;
}

double rotation_angle(const Mat3& r) {
    const double trace = r(0, 0) + r(1, 1) + r(2, 2);
    const double cos_theta = std::clamp((trace - 1.0) * 0.5, -1.0, 1.0);
    return std::acos(cos_theta);
}

}  // namespace pointforge
