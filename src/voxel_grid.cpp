#include "pointforge/voxel_grid.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace pointforge {
namespace {

// Индексът по всяка ос се затваря в 21 бита, тоест до 2097152 кубчета по ос.
// Три такива индекса се събират в един 64-битов ключ без сблъсък. Границата е
// проверена, а не приета: при твърде дребно ребро функцията отказва, вместо да
// върне мълчаливо сгрешен резултат от препълване.
constexpr std::uint64_t kAxisBits = 21;
constexpr std::uint64_t kMaxAxisCells = (std::uint64_t{1} << kAxisBits) - 1;

std::uint64_t cell_key(std::uint64_t ix, std::uint64_t iy, std::uint64_t iz) {
    return (ix << (2 * kAxisBits)) | (iy << kAxisBits) | iz;
}

std::uint64_t axis_cells(float extent, float leaf_size) {
    if (extent <= 0.0F) return 1;
    return static_cast<std::uint64_t>(std::floor(static_cast<double>(extent) / leaf_size)) + 1;
}

}  // namespace

VoxelGridShape voxel_grid_shape(const PointCloud& cloud, float leaf_size) {
    if (leaf_size <= 0.0F) throw std::invalid_argument("voxel_downsample: ребро <= 0");
    if (cloud.empty()) return {};
    const Aabb box = cloud.bounds();
    const Point3 e = box.extent();
    return {axis_cells(e.x, leaf_size), axis_cells(e.y, leaf_size), axis_cells(e.z, leaf_size)};
}

PointCloud voxel_downsample(const PointCloud& cloud, float leaf_size, VoxelGridStats* stats) {
    if (leaf_size <= 0.0F) throw std::invalid_argument("voxel_downsample: ребро <= 0");

    PointCloud out;
    if (stats != nullptr) {
        stats->input_points = cloud.size();
        stats->output_points = 0;
        stats->leaf_size = leaf_size;
    }
    if (cloud.empty()) return out;

    const VoxelGridShape shape = voxel_grid_shape(cloud, leaf_size);
    if (shape.nx > kMaxAxisCells || shape.ny > kMaxAxisCells || shape.nz > kMaxAxisCells) {
        throw std::invalid_argument("voxel_downsample: реброто дава над 2^21 кубчета по ос");
    }

    const Aabb box = cloud.bounds();
    const float inv_leaf = 1.0F / leaf_size;

    // Ключ и индекс на точка. Групирането става чрез сортиране, а не чрез
    // хеш-таблица: сортирането дава повторяем ред на изхода, а на тези размери
    // и по-добра локалност при обхождането.
    struct Entry {
        std::uint64_t key;
        std::uint32_t index;
    };
    std::vector<Entry> entries(cloud.size());

    const float* xs = cloud.xs();
    const float* ys = cloud.ys();
    const float* zs = cloud.zs();
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        const auto ix = static_cast<std::uint64_t>((xs[i] - box.min.x) * inv_leaf);
        const auto iy = static_cast<std::uint64_t>((ys[i] - box.min.y) * inv_leaf);
        const auto iz = static_cast<std::uint64_t>((zs[i] - box.min.z) * inv_leaf);
        entries[i] = {cell_key(std::min(ix, kMaxAxisCells), std::min(iy, kMaxAxisCells),
                               std::min(iz, kMaxAxisCells)),
                      static_cast<std::uint32_t>(i)};
    }

    std::sort(entries.begin(), entries.end(),
              [](const Entry& a, const Entry& b) { return a.key < b.key; });

    std::size_t begin = 0;
    while (begin < entries.size()) {
        std::size_t end = begin + 1;
        while (end < entries.size() && entries[end].key == entries[begin].key) ++end;

        double sx = 0.0;
        double sy = 0.0;
        double sz = 0.0;
        for (std::size_t i = begin; i < end; ++i) {
            const std::uint32_t p = entries[i].index;
            sx += xs[p];
            sy += ys[p];
            sz += zs[p];
        }
        const double n = static_cast<double>(end - begin);
        out.push_back(static_cast<float>(sx / n), static_cast<float>(sy / n),
                      static_cast<float>(sz / n));
        begin = end;
    }

    if (stats != nullptr) stats->output_points = out.size();
    return out;
}

}  // namespace pointforge
