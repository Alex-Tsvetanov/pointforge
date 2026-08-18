// transform.hpp: коравo преобразувание (ротация и транслация) и малката
// линейна алгебра, която му е нужна.
//
// Матриците са 3x3 и се пазят по редове. Проектът не внася библиотека за
// линейна алгебра: единствената нетривиална операция е сингулярното разлагане
// на матрица 3x3, което е под сто реда с едностранен метод на Якоби.
#ifndef POINTFORGE_TRANSFORM_HPP
#define POINTFORGE_TRANSFORM_HPP

#include <array>
#include <cmath>
#include <cstddef>

#include "pointforge/point_cloud.hpp"

namespace pointforge {

struct Mat3 {
    // m[ред][стълб]
    std::array<std::array<double, 3>, 3> m{{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};

    static Mat3 identity() { return Mat3{}; }

    static Mat3 zero() {
        Mat3 r;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) r.m[i][j] = 0.0;
        }
        return r;
    }

    double& operator()(int i, int j) { return m[i][j]; }
    double operator()(int i, int j) const { return m[i][j]; }

    Mat3 transposed() const {
        Mat3 r;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) r.m[i][j] = m[j][i];
        }
        return r;
    }

    double determinant() const {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
               m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
               m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }
};

inline Mat3 operator*(const Mat3& a, const Mat3& b) {
    Mat3 r = Mat3::zero();
    for (int i = 0; i < 3; ++i) {
        for (int k = 0; k < 3; ++k) {
            const double aik = a(i, k);
            for (int j = 0; j < 3; ++j) r(i, j) += aik * b(k, j);
        }
    }
    return r;
}

inline Point3 operator*(const Mat3& a, const Point3& p) {
    const double px = p.x;
    const double py = p.y;
    const double pz = p.z;
    return {static_cast<float>(a(0, 0) * px + a(0, 1) * py + a(0, 2) * pz),
            static_cast<float>(a(1, 0) * px + a(1, 1) * py + a(1, 2) * pz),
            static_cast<float>(a(2, 0) * px + a(2, 1) * py + a(2, 2) * pz)};
}

// Ротация около единичната ос (ax, ay, az) на ъгъл в радиани, по Родригес.
Mat3 rotation_axis_angle(double ax, double ay, double az, double angle_rad);

// Сингулярно разлагане A = U * S * V^T за матрица 3x3, едностранен Якоби.
struct Svd3 {
    Mat3 u;
    Mat3 v;
    std::array<double, 3> s{{0.0, 0.0, 0.0}};
};
Svd3 svd3(const Mat3& a);

struct RigidTransform {
    Mat3 rotation = Mat3::identity();
    Point3 translation{0.0F, 0.0F, 0.0F};

    static RigidTransform identity() { return RigidTransform{}; }

    Point3 apply(const Point3& p) const { return rotation * p + translation; }

    RigidTransform inverse() const {
        RigidTransform inv;
        inv.rotation = rotation.transposed();
        const Point3 t = inv.rotation * translation;
        inv.translation = {-t.x, -t.y, -t.z};
        return inv;
    }

    // Композиция: първо other, после текущото преобразувание.
    RigidTransform compose(const RigidTransform& other) const {
        RigidTransform r;
        r.rotation = rotation * other.rotation;
        r.translation = rotation * other.translation + translation;
        return r;
    }
};

// Прилага преобразуванието върху целия облак и връща нов облак. Стъпките в
// конвейера не променят входа си на място, вж. раздела за проектирането.
PointCloud transform_cloud(const PointCloud& cloud, const RigidTransform& t);

// Ъгъл на ротацията в радиани, от следата на матрицата. Ползва се за отчитане
// на остатъчната грешка спрямо еталонното преобразувание.
double rotation_angle(const Mat3& r);

}  // namespace pointforge

#endif  // POINTFORGE_TRANSFORM_HPP
