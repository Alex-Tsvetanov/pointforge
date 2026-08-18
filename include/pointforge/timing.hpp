// timing.hpp: измерване на време без външна библиотека.
//
// Часовникът е std::chrono::steady_clock, защото system_clock може да бъде
// преместен назад от синхронизация по време на измерването.
//
// Отчита се медианата, а не средното. Планировчикът на операционната система
// произвежда отделни много бавни изпълнения, а средното ги пренася върху целия
// резултат. Затова се пазят и минимумът, и максимумът: разликата между тях
// казва дали измерването изобщо е стабилно.
#ifndef POINTFORGE_TIMING_HPP
#define POINTFORGE_TIMING_HPP

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <vector>

namespace pointforge {

struct TimingStats {
    double median_ms = 0.0;
    double mean_ms = 0.0;
    double min_ms = 0.0;
    double max_ms = 0.0;
    int repetitions = 0;

    // Разсейване, отнесено към медианата. Над няколко процента резултатът не
    // е готов за таблица.
    double spread_ratio() const {
        return median_ms > 0.0 ? (max_ms - min_ms) / median_ms : 0.0;
    }
};

// Пречи на компилатора да изхвърли изчисление, чийто резултат не се ползва.
// Без това цял измерван цикъл може да изчезне и измерването да покаже нула.
template <class T>
inline void keep(const T& value) {
    // Празен asm блок с вход, но без изход, кара компилатора да смята
    // стойността за нужна, без да добавя инструкции.
#if defined(__GNUC__) || defined(__clang__)
    asm volatile("" : : "r,m"(value) : "memory");
#else
    volatile const T sink = value;
    (void)sink;
#endif
}

template <class Body>
TimingStats measure(int repetitions, int warmup, Body&& body) {
    for (int i = 0; i < warmup; ++i) body();

    std::vector<double> samples;
    samples.reserve(static_cast<std::size_t>(std::max(repetitions, 0)));
    for (int i = 0; i < repetitions; ++i) {
        const auto start = std::chrono::steady_clock::now();
        body();
        const auto stop = std::chrono::steady_clock::now();
        samples.push_back(std::chrono::duration<double, std::milli>(stop - start).count());
    }

    TimingStats stats;
    if (samples.empty()) return stats;

    std::sort(samples.begin(), samples.end());
    stats.repetitions = repetitions;
    stats.min_ms = samples.front();
    stats.max_ms = samples.back();
    const std::size_t mid = samples.size() / 2;
    stats.median_ms = (samples.size() % 2 == 1)
                          ? samples[mid]
                          : 0.5 * (samples[mid - 1] + samples[mid]);
    double sum = 0.0;
    for (const double s : samples) sum += s;
    stats.mean_ms = sum / static_cast<double>(samples.size());
    return stats;
}

}  // namespace pointforge

#endif  // POINTFORGE_TIMING_HPP
