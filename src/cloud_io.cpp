#include "pointforge/cloud_io.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <charconv>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <vector>

namespace pointforge {
namespace {

std::vector<std::string> split_words(const std::string& line) {
    std::vector<std::string> words;
    std::istringstream stream(line);
    std::string word;
    while (stream >> word) words.push_back(word);
    return words;
}

std::string lowercase(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

std::string extension_of(const std::string& path) {
    const std::size_t dot = path.find_last_of('.');
    if (dot == std::string::npos) return {};
    return lowercase(path.substr(dot + 1));
}

std::ifstream open_for_read(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) throw IoError("не може да се отвори за четене: " + path);
    return file;
}

std::ofstream open_for_write(const std::string& path) {
    std::ofstream file(path, std::ios::binary);
    if (!file) throw IoError("не може да се отвори за запис: " + path);
    return file;
}

// Общата част на двата четеца: тялото е таблица от числа и трябва да се вземат
// три определени стълба от нея.
struct ColumnLayout {
    int x = -1;
    int y = -1;
    int z = -1;
    int columns = 0;

    void validate(const std::string& format) const {
        if (x < 0 || y < 0 || z < 0) {
            throw IoError(format + ": липсва някое от полетата x, y или z");
        }
    }
};

PointCloud read_table(std::istream& in, const ColumnLayout& layout, std::size_t expected,
                      const std::string& format) {
    PointCloud cloud;
    cloud.reserve(expected);

    std::string line;
    std::vector<float> values;
    while (cloud.size() < expected && std::getline(in, line)) {
        if (line.empty()) continue;
        values.clear();
        std::istringstream stream(line);
        float value = 0.0F;
        while (stream >> value) values.push_back(value);
        if (values.empty()) continue;
        if (static_cast<int>(values.size()) < layout.columns) {
            throw IoError(format + ": ред с " + std::to_string(values.size()) + " стойности вместо " +
                          std::to_string(layout.columns));
        }
        cloud.push_back(values[static_cast<std::size_t>(layout.x)],
                        values[static_cast<std::size_t>(layout.y)],
                        values[static_cast<std::size_t>(layout.z)]);
    }

    if (cloud.size() != expected) {
        throw IoError(format + ": заглавието обявява " + std::to_string(expected) +
                      " точки, а в тялото има " + std::to_string(cloud.size()));
    }
    return cloud;
}

}  // namespace

PointCloud read_ply(const std::string& path) {
    std::ifstream file = open_for_read(path);

    std::string line;
    if (!std::getline(file, line)) throw IoError("PLY: празен файл: " + path);
    if (split_words(line).empty() || split_words(line)[0] != "ply") {
        throw IoError("PLY: липсва вълшебната дума ply в началото: " + path);
    }

    std::size_t vertex_count = 0;
    bool in_vertex_element = false;
    bool header_done = false;
    ColumnLayout layout;

    while (std::getline(file, line)) {
        const std::vector<std::string> words = split_words(line);
        if (words.empty()) continue;
        const std::string key = lowercase(words[0]);

        if (key == "format") {
            if (words.size() < 2 || lowercase(words[1]) != "ascii") {
                throw IoError("PLY: поддържа се само ascii вариантът, файлът е " +
                              (words.size() < 2 ? std::string("без формат") : words[1]));
            }
        } else if (key == "comment") {
            continue;
        } else if (key == "element") {
            if (words.size() < 3) throw IoError("PLY: непълен ред element");
            in_vertex_element = (lowercase(words[1]) == "vertex");
            if (in_vertex_element) vertex_count = std::stoull(words[2]);
        } else if (key == "property" && in_vertex_element) {
            // list-полетата в елемента vertex биха разбили табличния разбор.
            if (words.size() >= 2 && lowercase(words[1]) == "list") {
                throw IoError("PLY: списъчно поле в елемента vertex не се поддържа");
            }
            if (words.size() < 3) throw IoError("PLY: непълен ред property");
            const std::string name = lowercase(words[2]);
            if (name == "x") layout.x = layout.columns;
            if (name == "y") layout.y = layout.columns;
            if (name == "z") layout.z = layout.columns;
            ++layout.columns;
        } else if (key == "end_header") {
            header_done = true;
            break;
        }
    }

    if (!header_done) throw IoError("PLY: заглавието не завършва с end_header: " + path);
    layout.validate("PLY");
    return read_table(file, layout, vertex_count, "PLY");
}

void write_ply(const std::string& path, const PointCloud& cloud) {
    std::ofstream file = open_for_write(path);
    file << "ply\n";
    file << "format ascii 1.0\n";
    file << "comment written by PointForge\n";
    file << "element vertex " << cloud.size() << "\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "end_header\n";

    // Ръчно форматиране: std::ofstream с operator<< за float струва около
    // четири пъти повече време на точка от std::to_chars при милион точки.
    std::array<char, 64> buffer{};
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        const int written = std::snprintf(buffer.data(), buffer.size(), "%.9g %.9g %.9g\n",
                                          static_cast<double>(cloud.xs()[i]),
                                          static_cast<double>(cloud.ys()[i]),
                                          static_cast<double>(cloud.zs()[i]));
        if (written <= 0) throw IoError("PLY: неуспешно форматиране на точка");
        file.write(buffer.data(), written);
    }
    if (!file) throw IoError("PLY: неуспешен запис: " + path);
}

PointCloud read_pcd(const std::string& path) {
    std::ifstream file = open_for_read(path);

    std::size_t points = 0;
    std::size_t width = 0;
    std::size_t height = 1;
    bool has_points = false;
    bool data_seen = false;
    ColumnLayout layout;

    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty() && line[0] == '#') continue;
        const std::vector<std::string> words = split_words(line);
        if (words.empty()) continue;
        const std::string key = lowercase(words[0]);

        if (key == "fields") {
            layout.columns = 0;
            for (std::size_t i = 1; i < words.size(); ++i) {
                const std::string name = lowercase(words[i]);
                if (name == "x") layout.x = layout.columns;
                if (name == "y") layout.y = layout.columns;
                if (name == "z") layout.z = layout.columns;
                ++layout.columns;
            }
        } else if (key == "count") {
            for (std::size_t i = 1; i < words.size(); ++i) {
                if (words[i] != "1") throw IoError("PCD: поле с COUNT различно от 1 не се поддържа");
            }
        } else if (key == "points") {
            if (words.size() < 2) throw IoError("PCD: непълен ред POINTS");
            points = std::stoull(words[1]);
            has_points = true;
        } else if (key == "width") {
            if (words.size() >= 2) width = std::stoull(words[1]);
        } else if (key == "height") {
            if (words.size() >= 2) height = std::stoull(words[1]);
        } else if (key == "data") {
            if (words.size() < 2 || lowercase(words[1]) != "ascii") {
                throw IoError("PCD: поддържа се само DATA ascii");
            }
            data_seen = true;
            break;
        }
    }

    if (!data_seen) throw IoError("PCD: заглавието не стига до ред DATA: " + path);
    layout.validate("PCD");
    // POINTS не е задължителен във всички варианти на формата, WIDTH * HEIGHT е.
    const std::size_t expected = has_points ? points : width * height;
    return read_table(file, layout, expected, "PCD");
}

void write_pcd(const std::string& path, const PointCloud& cloud) {
    std::ofstream file = open_for_write(path);
    file << "# .PCD v0.7 - Point Cloud Data file format\n";
    file << "VERSION 0.7\n";
    file << "FIELDS x y z\n";
    file << "SIZE 4 4 4\n";
    file << "TYPE F F F\n";
    file << "COUNT 1 1 1\n";
    file << "WIDTH " << cloud.size() << "\n";
    file << "HEIGHT 1\n";
    file << "VIEWPOINT 0 0 0 1 0 0 0\n";
    file << "POINTS " << cloud.size() << "\n";
    file << "DATA ascii\n";

    std::array<char, 64> buffer{};
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        const int written = std::snprintf(buffer.data(), buffer.size(), "%.9g %.9g %.9g\n",
                                          static_cast<double>(cloud.xs()[i]),
                                          static_cast<double>(cloud.ys()[i]),
                                          static_cast<double>(cloud.zs()[i]));
        if (written <= 0) throw IoError("PCD: неуспешно форматиране на точка");
        file.write(buffer.data(), written);
    }
    if (!file) throw IoError("PCD: неуспешен запис: " + path);
}

PointCloud read_cloud(const std::string& path) {
    const std::string ext = extension_of(path);
    if (ext == "ply") return read_ply(path);
    if (ext == "pcd") return read_pcd(path);
    throw IoError("непознато разширение за облак от точки: " + path);
}

void write_cloud(const std::string& path, const PointCloud& cloud) {
    const std::string ext = extension_of(path);
    if (ext == "ply") {
        write_ply(path, cloud);
        return;
    }
    if (ext == "pcd") {
        write_pcd(path, cloud);
        return;
    }
    throw IoError("непознато разширение за облак от точки: " + path);
}

}  // namespace pointforge
