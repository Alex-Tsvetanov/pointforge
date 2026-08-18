// labeling.hpp: сегментация чрез свързани компоненти.
//
// Върху двоично изображение всяка свързана област получава свой номер, а за
// всяка се извежда и описание: площ, ограждащ правоъгълник и център на
// тежестта. Описанието е това, което следващият слой на веригата, изключен от
// обхвата, би приел за вход.
#ifndef POINTFORGE_LABELING_HPP
#define POINTFORGE_LABELING_HPP

#include <cstdint>
#include <vector>

#include "pointforge/image.hpp"

namespace pointforge {

enum class Connectivity {
    Four,   // само по ръб
    Eight,  // и по връх
};

struct Component {
    std::uint32_t label = 0;
    std::uint32_t area = 0;
    int min_x = 0;
    int min_y = 0;
    int max_x = 0;
    int max_y = 0;
    double centroid_x = 0.0;
    double centroid_y = 0.0;

    int width() const { return max_x - min_x + 1; }
    int height() const { return max_y - min_y + 1; }
};

struct LabelResult {
    int width = 0;
    int height = 0;
    // Номер на компонента за всеки пиксел, 0 за фон. Номерата започват от 1 и
    // са без прекъсвания.
    std::vector<std::uint32_t> labels;
    std::vector<Component> components;

    std::uint32_t label_at(int x, int y) const {
        return labels[static_cast<std::size_t>(y) * width + x];
    }
};

// Пиксел се брои за преден план, ако стойността му е различна от нула.
// Компонентите с площ под min_area се изхвърлят и пикселите им се връщат във
// фона, което премахва точковия шум без отделна стъпка за него.
LabelResult connected_components(const Image8& binary, Connectivity connectivity = Connectivity::Eight,
                                 std::uint32_t min_area = 1);

// Оцветяване на номерата за преглед. Цветовете се избират от номера по начин,
// който дава различни цветове на съседни номера, иначе картата излиза почти
// едноцветна.
Image8 colorize_labels(const LabelResult& result);

}  // namespace pointforge

#endif  // POINTFORGE_LABELING_HPP
