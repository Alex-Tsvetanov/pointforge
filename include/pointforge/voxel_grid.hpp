// voxel_grid.hpp: прореждане с вокселна решетка.
//
// Пространството се разделя на кубчета с ребро leaf_size, а всяко непразно
// кубче се свежда до една точка, центъра на тежестта на попадналите в него.
// Изборът на център на тежестта, а не на средата на кубчето, запазва
// геометрията на повърхнината: средата на кубчето внася грешка до половин
// ребро дори при точки, легнали точно върху равнина.
#ifndef POINTFORGE_VOXEL_GRID_HPP
#define POINTFORGE_VOXEL_GRID_HPP

#include <cstddef>
#include <cstdint>

#include "pointforge/point_cloud.hpp"

namespace pointforge {

struct VoxelGridStats {
    std::size_t input_points = 0;
    std::size_t output_points = 0;
    float leaf_size = 0.0F;

    double reduction_ratio() const {
        return input_points == 0 ? 0.0
                                 : static_cast<double>(output_points) / static_cast<double>(input_points);
    }
};

// Връща нов облак. Редът на изхода е определен от подредбата на кубчетата, а
// не от реда на входа, но е повторяем: две изпълнения върху един и същ вход
// дават еднакъв изход.
PointCloud voxel_downsample(const PointCloud& cloud, float leaf_size, VoxelGridStats* stats = nullptr);

// Броят кубчета по всяка ос при дадено ребро. Ползва се от отчета и от
// проверката за препълване.
struct VoxelGridShape {
    std::uint64_t nx = 0;
    std::uint64_t ny = 0;
    std::uint64_t nz = 0;
};
VoxelGridShape voxel_grid_shape(const PointCloud& cloud, float leaf_size);

}  // namespace pointforge

#endif  // POINTFORGE_VOXEL_GRID_HPP
