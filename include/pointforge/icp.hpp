// icp.hpp: съвместяване на два облака по схемата на итеративната най-близка
// точка с метрика точка към точка.
//
// Една итерация се състои от четири стъпки: намиране на съответствия чрез
// най-близък съсед в целевия облак, отхвърляне на съответствията над зададен
// праг по разстояние, оценяване на кораво преобразувание по метода на Кабш и
// прилагането му. Спирането е по три условия наведнъж: изчерпани итерации,
// пренебрежимо преобразувание на последната стъпка, или пренебрежима промяна
// в средноквадратичното отклонение.
#ifndef POINTFORGE_ICP_HPP
#define POINTFORGE_ICP_HPP

#include <cstddef>
#include <vector>

#include "pointforge/kdtree.hpp"
#include "pointforge/point_cloud.hpp"
#include "pointforge/transform.hpp"

namespace pointforge {

struct IcpOptions {
    int max_iterations = 50;

    // Съответствие с разстояние над този праг се отхвърля. Прагът е
    // задължителен, а не украса: без него всяка точка от източника получава
    // съответствие, включително точките без покритие в целевия облак, и те
    // издърпват решението.
    float max_correspondence_distance = 1.0F;

    // Спиране по големина на последното преобразувание.
    double rotation_epsilon = 1e-9;     // радиани
    double translation_epsilon = 1e-9;  // единици на облака

    // Спиране по промяна в средноквадратичното отклонение между две итерации.
    double rmse_epsilon = 1e-9;

    // Кой път за обхождане на лист ползва търсенето на съседи.
    NnPath path = NnPath::Batched;
};

struct IcpResult {
    RigidTransform transform = RigidTransform::identity();
    double rmse = 0.0;
    int iterations = 0;
    bool converged = false;
    std::size_t correspondences = 0;  // приети на последната итерация
};

// Съвместява source към target. Върнатото преобразувание е това, което трябва
// да се приложи върху source, за да легне върху target.
IcpResult icp_point_to_point(const PointCloud& source, const PointCloud& target,
                             const IcpOptions& options = {},
                             const RigidTransform& initial_guess = RigidTransform::identity());

// Вариант, който преизползва вече построен индекс над target. ICP е основният
// потребител на индекса и построяването му за всяко извикване е излишна
// работа, когато целевият облак не се променя.
IcpResult icp_point_to_point(const PointCloud& source, const PointCloud& target, const KdTree& target_index,
                             const IcpOptions& options = {},
                             const RigidTransform& initial_guess = RigidTransform::identity());

// Оценяване на кораво преобразувание по двойки съответстващи точки, метод на
// Кабш със сингулярно разлагане. Изнесено, защото се проверява самостоятелно.
// Изисква поне три двойки; при по-малко връща тъждественото преобразувание.
RigidTransform estimate_rigid_transform(const PointCloud& source, const PointCloud& target,
                                        const std::vector<std::uint32_t>& source_indices,
                                        const std::vector<std::uint32_t>& target_indices);

}  // namespace pointforge

#endif  // POINTFORGE_ICP_HPP
