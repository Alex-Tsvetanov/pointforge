// cloud_io.hpp: четене и запис на облаци от точки.
//
// Поддържат се два текстови формата: PLY в ascii вариант и PCD в ascii
// вариант. И двата се четат през общ разбор на заглавието, който намира
// стълбовете на x, y и z по име, вместо да разчита на реда им. Двоичните
// варианти на двата формата не се поддържат и се отхвърлят изрично, а не се
// четат наполовина.
#ifndef POINTFORGE_CLOUD_IO_HPP
#define POINTFORGE_CLOUD_IO_HPP

#include <stdexcept>
#include <string>

#include "pointforge/point_cloud.hpp"

namespace pointforge {

class IoError : public std::runtime_error {
public:
    explicit IoError(const std::string& message) : std::runtime_error(message) {}
};

PointCloud read_ply(const std::string& path);
void write_ply(const std::string& path, const PointCloud& cloud);

PointCloud read_pcd(const std::string& path);
void write_pcd(const std::string& path, const PointCloud& cloud);

// Избор на формат по разширението на файла. Неизвестно разширение е грешка,
// не мълчаливо предположение.
PointCloud read_cloud(const std::string& path);
void write_cloud(const std::string& path, const PointCloud& cloud);

}  // namespace pointforge

#endif  // POINTFORGE_CLOUD_IO_HPP
