#include "pointforge/labeling.hpp"

#include <algorithm>
#include <array>
#include <cstddef>

namespace pointforge {

LabelResult connected_components(const Image8& binary, Connectivity connectivity,
                                 std::uint32_t min_area) {
    LabelResult result;
    if (binary.empty()) return result;
    if (binary.channels() != 1) {
        throw std::invalid_argument("connected_components: очаква се един канал");
    }

    const int width = binary.width();
    const int height = binary.height();
    result.width = width;
    result.height = height;
    result.labels.assign(static_cast<std::size_t>(width) * height, 0U);

    static constexpr std::array<int, 8> kDx = {1, -1, 0, 0, 1, 1, -1, -1};
    static constexpr std::array<int, 8> kDy = {0, 0, 1, -1, 1, -1, 1, -1};
    const std::size_t neighbours = (connectivity == Connectivity::Four) ? 4 : 8;

    // Обхождане в ширина с явна опашка. Рекурсията тук е капан: една свързана
    // област може да покрие цялото изображение и дълбочината става равна на
    // броя на пикселите ѝ.
    std::vector<std::size_t> queue;
    std::vector<Component> collected;
    std::uint32_t next_label = 1;

    for (int y0 = 0; y0 < height; ++y0) {
        for (int x0 = 0; x0 < width; ++x0) {
            const std::size_t start = static_cast<std::size_t>(y0) * width + x0;
            if (binary.at(x0, y0) == 0 || result.labels[start] != 0) continue;

            const std::uint32_t label = next_label;
            Component component;
            component.label = label;
            component.min_x = x0;
            component.max_x = x0;
            component.min_y = y0;
            component.max_y = y0;

            double sum_x = 0.0;
            double sum_y = 0.0;

            queue.clear();
            queue.push_back(start);
            result.labels[start] = label;

            for (std::size_t head = 0; head < queue.size(); ++head) {
                const std::size_t index = queue[head];
                const int x = static_cast<int>(index % static_cast<std::size_t>(width));
                const int y = static_cast<int>(index / static_cast<std::size_t>(width));

                ++component.area;
                sum_x += x;
                sum_y += y;
                component.min_x = std::min(component.min_x, x);
                component.max_x = std::max(component.max_x, x);
                component.min_y = std::min(component.min_y, y);
                component.max_y = std::max(component.max_y, y);

                for (std::size_t n = 0; n < neighbours; ++n) {
                    const int nx = x + kDx[n];
                    const int ny = y + kDy[n];
                    if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
                    const std::size_t neighbour = static_cast<std::size_t>(ny) * width + nx;
                    if (result.labels[neighbour] != 0 || binary.at(nx, ny) == 0) continue;
                    result.labels[neighbour] = label;
                    queue.push_back(neighbour);
                }
            }

            component.centroid_x = sum_x / component.area;
            component.centroid_y = sum_y / component.area;

            if (component.area < min_area) {
                // Твърде малка: пикселите се връщат във фона и номерът не се
                // изразходва, за да останат номерата без прекъсвания.
                for (const std::size_t index : queue) result.labels[index] = 0;
            } else {
                collected.push_back(component);
                ++next_label;
            }
        }
    }

    result.components = std::move(collected);
    return result;
}

Image8 colorize_labels(const LabelResult& result) {
    Image8 out(result.width, result.height, 3);
    if (result.labels.empty()) return out;

    for (int y = 0; y < result.height; ++y) {
        for (int x = 0; x < result.width; ++x) {
            const std::uint32_t label = result.label_at(x, y);
            if (label == 0) continue;
            // Разбъркване на номера преди превръщането в цвят. Пряката
            // зависимост цвят(номер) дава почти еднакви цветове на съседни
            // номера, а точно те най-често са съседни и в изображението.
            std::uint32_t h = label * 2654435761U;
            h ^= h >> 15;
            out.at(x, y, 0) = static_cast<std::uint8_t>(60 + (h & 0xFFU) * 195 / 255);
            out.at(x, y, 1) = static_cast<std::uint8_t>(60 + ((h >> 8) & 0xFFU) * 195 / 255);
            out.at(x, y, 2) = static_cast<std::uint8_t>(60 + ((h >> 16) & 0xFFU) * 195 / 255);
        }
    }
    return out;
}

}  // namespace pointforge
