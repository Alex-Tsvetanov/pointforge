// text.hpp: подравняване на текст с кирилица в изхода на конзолата.
//
// Спецификаторите за ширина на printf броят БАЙТОВЕ. Заглавието на стълб на
// кирилица е два байта на буква в UTF-8, така че "%-8s" оставя стълба два пъти
// по-тесен, отколкото изглежда в изходния текст, и таблиците излизат разкривени.
// Затова подравняването се прави тук, по брой знаци, а на printf се подава вече
// подравнен низ.
#ifndef POINTFORGE_TEXT_HPP
#define POINTFORGE_TEXT_HPP

#include <cstddef>
#include <string>

namespace pointforge {

// Брой знаци в UTF-8 низ. Продължаващите байтове са тези с водещи битове 10.
inline std::size_t utf8_length(const std::string& text) {
    std::size_t count = 0;
    for (const char c : text) {
        if ((static_cast<unsigned char>(c) & 0xC0U) != 0x80U) ++count;
    }
    return count;
}

inline std::string pad_right(const std::string& text, std::size_t width) {
    const std::size_t length = utf8_length(text);
    return length >= width ? text : text + std::string(width - length, ' ');
}

inline std::string pad_left(const std::string& text, std::size_t width) {
    const std::size_t length = utf8_length(text);
    return length >= width ? text : std::string(width - length, ' ') + text;
}

}  // namespace pointforge

#endif  // POINTFORGE_TEXT_HPP
