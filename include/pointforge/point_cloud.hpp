// point_cloud.hpp: представяне на облак от точки, подредено по компонента.
//
// Координатите живеят в три отделни непрекъснати масива, а не в масив от тройки.
// Причината е достъпът при заявка за съседи: вътрешният цикъл чете само една
// координата на много точки наред, което е последователен достъп при това
// разположение и стъпка от 12 байта при масив от тройки.
#ifndef POINTFORGE_POINT_CLOUD_HPP
#define POINTFORGE_POINT_CLOUD_HPP

#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>
#include <vector>

namespace pointforge {

// Единична точка. Ползва се на границата на интерфейса, не за съхранение.
struct Point3 {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
};

inline Point3 operator-(const Point3& a, const Point3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Point3 operator+(const Point3& a, const Point3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }

inline float squared_distance(const Point3& a, const Point3& b) {
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    const float dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
}

// Изравнен по оси ограждащ паралелепипед.
struct Aabb {
    Point3 min{std::numeric_limits<float>::infinity(),
               std::numeric_limits<float>::infinity(),
               std::numeric_limits<float>::infinity()};
    Point3 max{-std::numeric_limits<float>::infinity(),
               -std::numeric_limits<float>::infinity(),
               -std::numeric_limits<float>::infinity()};

    bool empty() const { return min.x > max.x; }

    Point3 extent() const {
        if (empty()) return {0.0F, 0.0F, 0.0F};
        return {max.x - min.x, max.y - min.y, max.z - min.z};
    }

    // Индекс на най-дългата ос: 0 = x, 1 = y, 2 = z.
    int widest_axis() const {
        const Point3 e = extent();
        if (e.x >= e.y && e.x >= e.z) return 0;
        return (e.y >= e.z) ? 1 : 2;
    }
};

class PointCloud {
public:
    PointCloud() = default;

    std::size_t size() const { return x_.size(); }
    bool empty() const { return x_.empty(); }

    void clear() {
        x_.clear();
        y_.clear();
        z_.clear();
    }

    void reserve(std::size_t n) {
        x_.reserve(n);
        y_.reserve(n);
        z_.reserve(n);
    }

    void resize(std::size_t n) {
        x_.resize(n);
        y_.resize(n);
        z_.resize(n);
    }

    void push_back(const Point3& p) {
        x_.push_back(p.x);
        y_.push_back(p.y);
        z_.push_back(p.z);
    }

    void push_back(float px, float py, float pz) {
        x_.push_back(px);
        y_.push_back(py);
        z_.push_back(pz);
    }

    Point3 point(std::size_t i) const { return {x_[i], y_[i], z_[i]}; }

    void set_point(std::size_t i, const Point3& p) {
        x_[i] = p.x;
        y_[i] = p.y;
        z_[i] = p.z;
    }

    // Достъп до отделните масиви. Векторизираният път ги ползва пряко, затова
    // указателите са част от договора, а не подробност от реализацията.
    const float* xs() const { return x_.data(); }
    const float* ys() const { return y_.data(); }
    const float* zs() const { return z_.data(); }
    float* xs() { return x_.data(); }
    float* ys() { return y_.data(); }
    float* zs() { return z_.data(); }

    Aabb bounds() const {
        Aabb box;
        for (std::size_t i = 0; i < x_.size(); ++i) {
            box.min.x = std::min(box.min.x, x_[i]);
            box.min.y = std::min(box.min.y, y_[i]);
            box.min.z = std::min(box.min.z, z_[i]);
            box.max.x = std::max(box.max.x, x_[i]);
            box.max.y = std::max(box.max.y, y_[i]);
            box.max.z = std::max(box.max.z, z_[i]);
        }
        return box;
    }

    Point3 centroid() const {
        if (x_.empty()) return {0.0F, 0.0F, 0.0F};
        // Сумиране в double: при 10^6 точки натрупаната грешка във float вече
        // измества центъра достатъчно, за да провали проверката на ICP.
        double sx = 0.0;
        double sy = 0.0;
        double sz = 0.0;
        for (std::size_t i = 0; i < x_.size(); ++i) {
            sx += x_[i];
            sy += y_[i];
            sz += z_[i];
        }
        const double n = static_cast<double>(x_.size());
        return {static_cast<float>(sx / n), static_cast<float>(sy / n), static_cast<float>(sz / n)};
    }

    // Байтове, заети от самите координати. Ползва се в отчета за паметта.
    std::size_t storage_bytes() const { return 3 * x_.capacity() * sizeof(float); }

private:
    std::vector<float> x_;
    std::vector<float> y_;
    std::vector<float> z_;
};

}  // namespace pointforge

#endif  // POINTFORGE_POINT_CLOUD_HPP
