#include <exception>
#include <iostream>

#include "test_framework.hpp"

namespace pftest {

std::vector<TestCase>& registry() {
    static std::vector<TestCase> cases;
    return cases;
}

int& current_failures() {
    static int failures = 0;
    return failures;
}

void report_failure(const char* file, int line, const std::string& what) {
    ++current_failures();
    std::cout << "    ! " << file << ":" << line << ": " << what << "\n";
}

int run(int argc, char** argv) {
    // Без буфериране: при пренасочен изход буферът се губи, ако проверката
    // прекрати процеса, и в дневника не остава коя точно е паднала.
    std::cout << std::unitbuf;

    // Един незадължителен довод: име на група. CTest подава по едно име на
    // запис, а без довод се пускат всички групи.
    const std::string filter = (argc > 1) ? argv[1] : std::string();

    int total = 0;
    int failed = 0;
    for (const TestCase& test : registry()) {
        if (!filter.empty() && test.suite != filter) continue;
        ++total;
        current_failures() = 0;
        std::cout << "[ РАБОТИ  ] " << test.suite << "." << test.name << "\n";
        try {
            test.fn();
        } catch (const std::exception& error) {
            report_failure("<изключение>", 0, error.what());
        } catch (...) {
            report_failure("<изключение>", 0, "непознат тип");
        }
        if (current_failures() == 0) {
            std::cout << "[      ОК ] " << test.suite << "." << test.name << "\n";
        } else {
            ++failed;
            std::cout << "[  ПАДНА  ] " << test.suite << "." << test.name << " ("
                      << current_failures() << " проверки)\n";
        }
    }

    if (total == 0) {
        std::cout << "няма проверки за група \"" << filter << "\"\n";
        return 2;
    }
    std::cout << "\n" << (total - failed) << " от " << total << " минаха\n";
    return failed == 0 ? 0 : 1;
}

}  // namespace pftest

int main(int argc, char** argv) { return pftest::run(argc, argv); }
