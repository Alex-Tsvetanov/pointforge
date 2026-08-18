// test_framework.hpp: минималният набор от проверки, който проектът ползва.
//
// Съществува, за да няма проверката външна зависимост. Регистрацията е през
// глобален обект, конструиран преди main, а изборът на група идва от
// командния ред, което е достатъчно, за да се закачи всяка група като отделен
// запис в CTest.
#ifndef POINTFORGE_TEST_FRAMEWORK_HPP
#define POINTFORGE_TEST_FRAMEWORK_HPP

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

namespace pftest {

using TestFn = void (*)();

struct TestCase {
    std::string suite;
    std::string name;
    TestFn fn;
};

std::vector<TestCase>& registry();
int& current_failures();
void report_failure(const char* file, int line, const std::string& what);
int run(int argc, char** argv);

struct Registrar {
    Registrar(const char* suite, const char* name, TestFn fn) {
        registry().push_back({suite, name, fn});
    }
};

// Превръща стойност в текст за съобщението при провал. Не всичко, което може
// да се сравнява, може и да се извежда, затова изводимостта се проверява, а
// контейнерите се изброяват поелементно.
template <class T>
std::string show(const T& value) {
    std::ostringstream stream;
    if constexpr (requires { stream << value; }) {
        stream << value;
    } else if constexpr (requires {
                             value.size();
                             value.begin();
                         }) {
        stream << "[" << value.size() << ":";
        std::size_t shown = 0;
        for (const auto& element : value) {
            if (shown++ == 8) {
                stream << " ...";
                break;
            }
            stream << " " << show(element);
        }
        stream << "]";
    } else {
        stream << "<не се извежда>";
    }
    return stream.str();
}

}  // namespace pftest

#define PF_CONCAT_INNER(a, b) a##b
#define PF_CONCAT(a, b) PF_CONCAT_INNER(a, b)

#define PF_TEST(suite_name, test_name)                                                        \
    static void PF_CONCAT(pf_body_, __LINE__)();                                              \
    static const ::pftest::Registrar PF_CONCAT(pf_reg_, __LINE__)(#suite_name, #test_name,    \
                                                                  &PF_CONCAT(pf_body_, __LINE__)); \
    static void PF_CONCAT(pf_body_, __LINE__)()

#define PF_CHECK(condition)                                                          \
    do {                                                                             \
        if (!(condition)) ::pftest::report_failure(__FILE__, __LINE__, #condition);   \
    } while (false)

#define PF_CHECK_EQ(lhs, rhs)                                                                    \
    do {                                                                                         \
        const auto pf_l = (lhs);                                                                 \
        const auto pf_r = (rhs);                                                                 \
        if (!(pf_l == pf_r)) {                                                                   \
            ::pftest::report_failure(__FILE__, __LINE__,                                         \
                                     std::string(#lhs " == " #rhs " | лява: ") +                 \
                                         ::pftest::show(pf_l) + ", дясна: " +                    \
                                         ::pftest::show(pf_r));                                  \
        }                                                                                        \
    } while (false)

#define PF_CHECK_NEAR(lhs, rhs, tolerance)                                                       \
    do {                                                                                         \
        const double pf_l = static_cast<double>(lhs);                                            \
        const double pf_r = static_cast<double>(rhs);                                            \
        const double pf_t = static_cast<double>(tolerance);                                      \
        if (!(std::fabs(pf_l - pf_r) <= pf_t)) {                                                 \
            ::pftest::report_failure(__FILE__, __LINE__,                                         \
                                     std::string(#lhs " ~ " #rhs " | лява: ") +                  \
                                         ::pftest::show(pf_l) + ", дясна: " +                    \
                                         ::pftest::show(pf_r) + ", допуск: " +                   \
                                         ::pftest::show(pf_t));                                  \
        }                                                                                        \
    } while (false)

#define PF_CHECK_THROWS(expression)                                                  \
    do {                                                                             \
        bool pf_threw = false;                                                       \
        try {                                                                        \
            (void)(expression);                                                      \
        } catch (...) {                                                              \
            pf_threw = true;                                                         \
        }                                                                            \
        if (!pf_threw)                                                               \
            ::pftest::report_failure(__FILE__, __LINE__, #expression " не хвърли");   \
    } while (false)

#endif  // POINTFORGE_TEST_FRAMEWORK_HPP
