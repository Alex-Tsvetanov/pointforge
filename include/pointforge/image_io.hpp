// image_io.hpp: четене и запис на PGM и PPM в двоичния им вариант, P5 и P6.
//
// Форматите са избрани, защото се четат и записват без външна библиотека и се
// отварят от всеки преглед на изображения. Заглавието им допуска коментари и
// произволен вид празно пространство между полетата, което е точното място,
// където наивните четци се чупят, затова разборът минава през общ четец на
// лексема, а не през operator>> върху целия ред.
#ifndef POINTFORGE_IMAGE_IO_HPP
#define POINTFORGE_IMAGE_IO_HPP

#include <string>

#include "pointforge/cloud_io.hpp"  // IoError
#include "pointforge/image.hpp"

namespace pointforge {

// Чете P5 (полутоново) или P6 (цветно) и връща изображение съответно с един
// или с три канала.
Image8 read_pnm(const std::string& path);

// Записва един канал като P5. Изображение с три канала се отхвърля, вместо да
// бъде мълчаливо превърнато.
void write_pgm(const std::string& path, const Image8& image);

// Записва три канала като P6. Изображение с един канал се повтаря по трите
// канала, защото това е еднозначно.
void write_ppm(const std::string& path, const Image8& image);

}  // namespace pointforge

#endif  // POINTFORGE_IMAGE_IO_HPP
