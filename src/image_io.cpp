#include "pointforge/image_io.hpp"

#include <cctype>
#include <fstream>
#include <string>

namespace pointforge {
namespace {

// Чете следващата лексема от заглавието, прескачайки празно пространство и
// коментари. Коментарът започва с # и стига до края на реда, и по стандарта
// може да се появи навсякъде в заглавието, включително по средата на число.
std::string next_header_token(std::istream& in) {
    std::string token;
    int c = in.get();
    while (c != EOF) {
        if (c == '#') {
            while (c != EOF && c != '\n') c = in.get();
            continue;
        }
        if (std::isspace(static_cast<unsigned char>(c)) == 0) break;
        c = in.get();
    }
    while (c != EOF && std::isspace(static_cast<unsigned char>(c)) == 0 && c != '#') {
        token.push_back(static_cast<char>(c));
        c = in.get();
    }
    // Знакът, който прекрати лексемата, се връща в потока. Иначе разделителят
    // след максималната стойност вече е изяден и проверката за него по-долу
    // отхвърля напълно редовен файл.
    if (c != EOF) in.unget();
    return token;
}

int parse_positive(const std::string& token, const std::string& what) {
    if (token.empty()) throw IoError("PNM: липсва " + what);
    try {
        const int value = std::stoi(token);
        if (value <= 0) throw IoError("PNM: " + what + " не е положително число");
        return value;
    } catch (const std::invalid_argument&) {
        throw IoError("PNM: " + what + " не е число: " + token);
    } catch (const std::out_of_range&) {
        throw IoError("PNM: " + what + " е извън обхвата: " + token);
    }
}

}  // namespace

Image8 read_pnm(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) throw IoError("не може да се отвори за четене: " + path);

    const std::string magic = next_header_token(file);
    int channels = 0;
    if (magic == "P5") {
        channels = 1;
    } else if (magic == "P6") {
        channels = 3;
    } else {
        throw IoError("PNM: поддържат се само P5 и P6, файлът е " + magic);
    }

    const int width = parse_positive(next_header_token(file), "ширина");
    const int height = parse_positive(next_header_token(file), "височина");
    const int max_value = parse_positive(next_header_token(file), "максимална стойност");
    if (max_value > 255) throw IoError("PNM: максимална стойност над 255 не се поддържа");

    // Точно един разделител след максималната стойност, после суровите байтове.
    const int separator = file.get();
    if (separator == EOF || std::isspace(static_cast<unsigned char>(separator)) == 0) {
        throw IoError("PNM: липсва разделител между заглавието и данните");
    }

    Image8 image(width, height, channels);
    const std::streamsize expected =
        static_cast<std::streamsize>(width) * height * channels;
    file.read(reinterpret_cast<char*>(image.data().data()), expected);
    if (file.gcount() != expected) {
        throw IoError("PNM: тялото е " + std::to_string(file.gcount()) + " байта вместо " +
                      std::to_string(expected));
    }

    // Стойностите се мащабират само ако максимумът не е 255. Пропускането на
    // тази стъпка е обичайната причина изображение с max=1 да излезе черно.
    if (max_value != 255) {
        for (std::uint8_t& v : image.data()) {
            v = static_cast<std::uint8_t>((static_cast<int>(v) * 255) / max_value);
        }
    }
    return image;
}

void write_pgm(const std::string& path, const Image8& image) {
    if (image.channels() != 1) throw IoError("PGM: очаква се изображение с един канал");
    std::ofstream file(path, std::ios::binary);
    if (!file) throw IoError("не може да се отвори за запис: " + path);
    file << "P5\n" << image.width() << " " << image.height() << "\n255\n";
    file.write(reinterpret_cast<const char*>(image.data().data()),
               static_cast<std::streamsize>(image.data().size()));
    if (!file) throw IoError("PGM: неуспешен запис: " + path);
}

void write_ppm(const std::string& path, const Image8& image) {
    if (image.channels() != 1 && image.channels() != 3) {
        throw IoError("PPM: очаква се изображение с един или три канала");
    }
    std::ofstream file(path, std::ios::binary);
    if (!file) throw IoError("не може да се отвори за запис: " + path);
    file << "P6\n" << image.width() << " " << image.height() << "\n255\n";

    if (image.channels() == 3) {
        file.write(reinterpret_cast<const char*>(image.data().data()),
                   static_cast<std::streamsize>(image.data().size()));
    } else {
        std::vector<std::uint8_t> row(static_cast<std::size_t>(image.width()) * 3);
        for (int y = 0; y < image.height(); ++y) {
            for (int x = 0; x < image.width(); ++x) {
                const std::uint8_t v = image.at(x, y);
                row[static_cast<std::size_t>(x) * 3 + 0] = v;
                row[static_cast<std::size_t>(x) * 3 + 1] = v;
                row[static_cast<std::size_t>(x) * 3 + 2] = v;
            }
            file.write(reinterpret_cast<const char*>(row.data()),
                       static_cast<std::streamsize>(row.size()));
        }
    }
    if (!file) throw IoError("PPM: неуспешен запис: " + path);
}

}  // namespace pointforge
