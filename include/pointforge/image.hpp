// image.hpp: двата растерни типа на проекта.
//
// Image8 е това, което се чете и записва: осем бита на канал, един канал за
// полутоново и три за цветно изображение. ImageF е това, върху което се смята:
// градиентите и изгладените стойности излизат извън диапазона 0..255 и
// закръгляването между стъпките на конвейера трупа грешка. Затова целият път
// от изглаждането до потискането на немаксимумите тече в ImageF, а обратно към
// осем бита се минава само на изхода.
#ifndef POINTFORGE_IMAGE_HPP
#define POINTFORGE_IMAGE_HPP

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace pointforge {

class Image8 {
public:
    Image8() = default;
    Image8(int width, int height, int channels)
        : width_(width), height_(height), channels_(channels),
          data_(static_cast<std::size_t>(width) * height * channels, 0) {
        if (width < 0 || height < 0 || channels < 1 || channels > 4) {
            throw std::invalid_argument("Image8: недопустими размери");
        }
    }

    int width() const { return width_; }
    int height() const { return height_; }
    int channels() const { return channels_; }
    bool empty() const { return data_.empty(); }
    std::size_t pixel_count() const { return static_cast<std::size_t>(width_) * height_; }

    std::uint8_t& at(int x, int y, int c = 0) {
        return data_[(static_cast<std::size_t>(y) * width_ + x) * channels_ + c];
    }
    std::uint8_t at(int x, int y, int c = 0) const {
        return data_[(static_cast<std::size_t>(y) * width_ + x) * channels_ + c];
    }

    bool in_bounds(int x, int y) const { return x >= 0 && y >= 0 && x < width_ && y < height_; }

    std::vector<std::uint8_t>& data() { return data_; }
    const std::vector<std::uint8_t>& data() const { return data_; }

private:
    int width_ = 0;
    int height_ = 0;
    int channels_ = 1;
    std::vector<std::uint8_t> data_;
};

class ImageF {
public:
    ImageF() = default;
    ImageF(int width, int height)
        : width_(width), height_(height), data_(static_cast<std::size_t>(width) * height, 0.0F) {
        if (width < 0 || height < 0) throw std::invalid_argument("ImageF: недопустими размери");
    }

    int width() const { return width_; }
    int height() const { return height_; }
    bool empty() const { return data_.empty(); }

    float& at(int x, int y) { return data_[static_cast<std::size_t>(y) * width_ + x]; }
    float at(int x, int y) const { return data_[static_cast<std::size_t>(y) * width_ + x]; }

    // Достъп със захващане на координатите към ръба. Всички ядра ползват този
    // достъп, за да е обработката на границата на едно място, а не преписана
    // във всяка функция.
    float clamped(int x, int y) const {
        const int cx = std::clamp(x, 0, width_ - 1);
        const int cy = std::clamp(y, 0, height_ - 1);
        return data_[static_cast<std::size_t>(cy) * width_ + cx];
    }

    std::vector<float>& data() { return data_; }
    const std::vector<float>& data() const { return data_; }

private:
    int width_ = 0;
    int height_ = 0;
    std::vector<float> data_;
};

// Преобразувания между двата типа. to_image8 отрязва към 0..255 със
// закръгляване, а не с отрязване надолу.
ImageF to_float(const Image8& image, int channel = 0);
Image8 to_image8(const ImageF& image);

// Полутоново от цветно по препоръчаните тегла за яркост. Един канал влиза
// непроменен.
Image8 to_grayscale(const Image8& image);

// Мащабира стойностите към 0..255 по действителния си минимум и максимум.
// Ползва се за записване на градиенти, чийто диапазон не е известен
// предварително.
Image8 normalize_to_image8(const ImageF& image);

}  // namespace pointforge

#endif  // POINTFORGE_IMAGE_HPP
