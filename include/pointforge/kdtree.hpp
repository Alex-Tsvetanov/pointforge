// kdtree.hpp: пространствен индекс над облак от точки.
//
// Интерфейсът е умишлено тесен: построяване, заявка за k най-близки съседи,
// заявка по радиус и отчет за паметта. Всичко останало в проекта е потребител
// на този интерфейс.
//
// Двата пътя за изпълнение на листата, скаларен и пакетен, са на разположение
// през един и същ метод и се избират с параметър. Така една и съща заявка се
// изпълнява и по двата начина върху едни и същи данни, което е условието
// сравнението между тях да е измерване, а не твърдение.
#ifndef POINTFORGE_KDTREE_HPP
#define POINTFORGE_KDTREE_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

#include "pointforge/point_cloud.hpp"

namespace pointforge {

// Съсед: индекс В ИЗХОДНИЯ облак и квадрат на разстоянието. Квадратът, а не
// разстоянието, за да няма корен във вътрешния цикъл.
struct Neighbor {
    std::uint32_t index = 0;
    float squared_distance = 0.0F;
};

// Кой път се ползва при обхождането на лист.
enum class NnPath {
    Scalar,   // точка по точка, с проверка след всяка
    Batched,  // на блокове от kBlockSize, разстоянията се смятат без разклонения
};

struct KdTreeOptions {
    // Максимален брой точки в лист. Балансира дълбочината на обхождането срещу
    // дължината на векторизирания цикъл, затова е настройка, а не константа.
    std::uint32_t leaf_size = 32;
};

class KdTree {
public:
    // Дължина на блока в пакетния път. Кратна на ширината на векторния
    // регистър при float за AVX2 (8) и за SSE (4).
    static constexpr std::uint32_t kBlockSize = 16;

    KdTree() = default;
    explicit KdTree(const PointCloud& cloud, KdTreeOptions options = {});

    void build(const PointCloud& cloud, KdTreeOptions options = {});

    std::size_t size() const { return px_.size(); }
    bool empty() const { return px_.empty(); }
    std::size_t node_count() const { return nodes_.size(); }
    std::uint32_t leaf_size() const { return options_.leaf_size; }

    // k най-близки съседи на query. Резултатът е подреден по нарастващо
    // разстояние. Върнатите индекси сочат в облака, подаден на build.
    void knn(const Point3& query, std::size_t k, std::vector<Neighbor>& out,
             NnPath path = NnPath::Batched) const;

    std::vector<Neighbor> knn(const Point3& query, std::size_t k, NnPath path = NnPath::Batched) const;

    // Всички съседи на разстояние до radius включително. Резултатът НЕ е
    // подреден: подреждането не е част от договора и струва време, което
    // повечето потребители не искат да платят.
    void radius_search(const Point3& query, float radius, std::vector<Neighbor>& out,
                       NnPath path = NnPath::Batched) const;

    std::vector<Neighbor> radius_search(const Point3& query, float radius,
                                        NnPath path = NnPath::Batched) const;

    // Единственият най-близък съсед. Отделен път, защото ICP го вика веднъж на
    // точка на итерация и купчината за k = 1 е чиста излишна работа.
    // Връща false само за празно дърво.
    bool nearest(const Point3& query, Neighbor& out, NnPath path = NnPath::Batched) const;

    // Памет, заета от индекса: разбърканите координати, картата на индексите и
    // възлите. Отчита се capacity, не size, защото това е заетото.
    std::size_t memory_bytes() const;

private:
    struct Node {
        float split = 0.0F;
        std::int32_t axis = -1;  // -1 означава лист
        std::uint32_t begin = 0;
        std::uint32_t end = 0;
        std::uint32_t right_child = 0;  // левият е винаги следващият в масива
    };

    std::uint32_t build_recursive(std::vector<std::uint32_t>& order, std::uint32_t begin,
                                  std::uint32_t end, const PointCloud& cloud);

    // Координати, пренаредени в реда на обхождане на дървото, така че точките
    // на един лист да са съседни в паметта. Това е предпоставката пакетният
    // път изобщо да има смисъл.
    std::vector<float> px_;
    std::vector<float> py_;
    std::vector<float> pz_;
    std::vector<std::uint32_t> original_index_;
    std::vector<Node> nodes_;
    KdTreeOptions options_{};
};

// Изчерпателно търсене. Точно по определение, служи за еталон при проверката
// на индекса и е бавно нарочно.
std::vector<Neighbor> brute_force_knn(const PointCloud& cloud, const Point3& query, std::size_t k);
std::vector<Neighbor> brute_force_radius(const PointCloud& cloud, const Point3& query, float radius);

}  // namespace pointforge

#endif  // POINTFORGE_KDTREE_HPP
