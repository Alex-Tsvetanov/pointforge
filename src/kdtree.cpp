#include "pointforge/kdtree.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

#include <xsimd/xsimd.hpp>

#if !defined(XSIMD_NO_SUPPORTED_ARCHITECTURE)
#define POINTFORGE_HAS_XSIMD 1
#endif

namespace pointforge {

bool simd_leaf_scan_available() noexcept {
#if defined(POINTFORGE_HAS_XSIMD)
    return true;
#else
    return false;
#endif
}

namespace {

// Купчина с най-лошия елемент отгоре, ограничена до k елемента. Държи се в
// подадения вектор, за да може повикващият да преизползва паметта между
// заявките: ICP прави по една заявка на точка и алокацията там доминира.
struct BoundedHeap {
    std::vector<Neighbor>& items;
    std::size_t capacity;

    static bool worse(const Neighbor& a, const Neighbor& b) {
        return a.squared_distance < b.squared_distance;
    }

    bool full() const { return items.size() >= capacity; }

    float worst() const {
        return items.size() < capacity ? std::numeric_limits<float>::infinity()
                                       : items.front().squared_distance;
    }

    void offer(std::uint32_t index, float d2) {
        if (items.size() < capacity) {
            items.push_back({index, d2});
            std::push_heap(items.begin(), items.end(), worse);
        } else if (d2 < items.front().squared_distance) {
            std::pop_heap(items.begin(), items.end(), worse);
            items.back() = {index, d2};
            std::push_heap(items.begin(), items.end(), worse);
        }
    }
};

}  // namespace

KdTree::KdTree(const PointCloud& cloud, KdTreeOptions options) { build(cloud, options); }

void KdTree::build(const PointCloud& cloud, KdTreeOptions options) {
    options_ = options;
    if (options_.leaf_size == 0) options_.leaf_size = 1;

    px_.clear();
    py_.clear();
    pz_.clear();
    original_index_.clear();
    nodes_.clear();

    const std::uint32_t n = static_cast<std::uint32_t>(cloud.size());
    if (n == 0) return;

    std::vector<std::uint32_t> order(n);
    std::iota(order.begin(), order.end(), 0U);

    px_.resize(n);
    py_.resize(n);
    pz_.resize(n);
    original_index_.resize(n);

    // Нареждането се строи върху order, а разбъркването на координатите става
    // наведнъж накрая. Разместване по време на построяването би струвало по
    // три записа на точка на всяко ниво.
    nodes_.reserve(2 * (n / options_.leaf_size + 1));
    build_recursive(order, 0, n, cloud);

    for (std::uint32_t i = 0; i < n; ++i) {
        const std::uint32_t src = order[i];
        px_[i] = cloud.xs()[src];
        py_[i] = cloud.ys()[src];
        pz_[i] = cloud.zs()[src];
        original_index_[i] = src;
    }
}

std::uint32_t KdTree::build_recursive(std::vector<std::uint32_t>& order, std::uint32_t begin,
                                      std::uint32_t end, const PointCloud& cloud) {
    const std::uint32_t node_index = static_cast<std::uint32_t>(nodes_.size());
    nodes_.push_back(Node{});

    const std::uint32_t count = end - begin;
    if (count <= options_.leaf_size) {
        Node& leaf = nodes_[node_index];
        leaf.axis = -1;
        leaf.begin = begin;
        leaf.end = end;
        return node_index;
    }

    // Разделяне по най-дългата ос на текущото подмножество, а не по редуващи
    // се оси. При издължена сцена редуването дава дълги тесни клетки, а те
    // правят отсичането безполезно.
    Aabb box;
    for (std::uint32_t i = begin; i < end; ++i) {
        const std::uint32_t s = order[i];
        const float x = cloud.xs()[s];
        const float y = cloud.ys()[s];
        const float z = cloud.zs()[s];
        box.min.x = std::min(box.min.x, x);
        box.min.y = std::min(box.min.y, y);
        box.min.z = std::min(box.min.z, z);
        box.max.x = std::max(box.max.x, x);
        box.max.y = std::max(box.max.y, y);
        box.max.z = std::max(box.max.z, z);
    }
    const int axis = box.widest_axis();
    const float* coord = (axis == 0) ? cloud.xs() : (axis == 1) ? cloud.ys() : cloud.zs();

    const std::uint32_t mid = begin + count / 2;
    std::nth_element(order.begin() + begin, order.begin() + mid, order.begin() + end,
                     [coord](std::uint32_t a, std::uint32_t b) { return coord[a] < coord[b]; });
    const float split = coord[order[mid]];

    // Всички точки с еднаква координата по оста: разделянето по медиана не
    // напредва и рекурсията не би свършила. Върши работа лист, дори голям.
    if (split == coord[order[begin]] && split == coord[order[end - 1]]) {
        Node& leaf = nodes_[node_index];
        leaf.axis = -1;
        leaf.begin = begin;
        leaf.end = end;
        return node_index;
    }

    build_recursive(order, begin, mid, cloud);
    const std::uint32_t right = build_recursive(order, mid, end, cloud);

    Node& node = nodes_[node_index];
    node.axis = axis;
    node.split = split;
    node.right_child = right;
    return node_index;
}

// --- обхождане на лист --------------------------------------------------

namespace {

struct LeafScanContext {
    const float* px;
    const float* py;
    const float* pz;
    const std::uint32_t* original;
    float qx;
    float qy;
    float qz;
};

// LISTING_BEGIN scan_leaf_scalar
void scan_leaf_scalar(const LeafScanContext& c, std::uint32_t begin, std::uint32_t end,
                      BoundedHeap& heap) {
    for (std::uint32_t i = begin; i < end; ++i) {
        const float dx = c.px[i] - c.qx;
        const float dy = c.py[i] - c.qy;
        const float dz = c.pz[i] - c.qz;
        const float d2 = dx * dx + dy * dy + dz * dz;
        if (d2 < heap.worst() || !heap.full()) heap.offer(c.original[i], d2);
    }
}
// LISTING_END scan_leaf_scalar

// Пълни distances[0..n) със squared distance. При n == kBlockSize и наличен
// xsimd пътят ползва xsimd::batch; иначе същият скаларен цикъл като пакетния.
void fill_block_distances(const float* bx, const float* by, const float* bz, float qx, float qy,
                          float qz, float* distances, std::uint32_t n, bool use_simd) {
#if defined(POINTFORGE_HAS_XSIMD)
    if (use_simd && n == KdTree::kBlockSize) {
        using batch = xsimd::batch<float>;
        static_assert(KdTree::kBlockSize % batch::size == 0,
                      "kBlockSize must be a multiple of xsimd::batch<float>::size");
        const batch vqx(qx);
        const batch vqy(qy);
        const batch vqz(qz);
        for (std::uint32_t j = 0; j < KdTree::kBlockSize; j += static_cast<std::uint32_t>(batch::size)) {
            const batch dx = xsimd::load_unaligned(bx + j) - vqx;
            const batch dy = xsimd::load_unaligned(by + j) - vqy;
            const batch dz = xsimd::load_unaligned(bz + j) - vqz;
            // fma(a,a, fma(b,b, c*c)) == a*a + b*b + c*c; xsimd избира FMA когато
            // архитектурата го има, иначе еквивалентни mul/add.
            const batch d2 = xsimd::fma(dx, dx, xsimd::fma(dy, dy, dz * dz));
            xsimd::store_unaligned(distances + j, d2);
        }
        return;
    }
#else
    (void)use_simd;
#endif

    for (std::uint32_t j = 0; j < n; ++j) {
        const float dx = bx[j] - qx;
        const float dy = by[j] - qy;
        const float dz = bz[j] - qz;
        distances[j] = dx * dx + dy * dy + dz * dz;
    }
}

// Пакетен път. Разстоянията за целия блок се смятат в цикъл без разклонения и
// без обръщение към купчината. Това е формата, която компилаторът може да
// векторизира сам; NnPath::Simd ползва xsimd за същата сметка.
//
// Обновяването на купчината е втори цикъл. Преди него блокът се отхвърля
// изцяло, ако и най-близката му точка е по-далече от най-лошата приета, което
// при пълна купчина спестява повечето обръщения към нея.
// LISTING_BEGIN scan_leaf_batched
void scan_leaf_batched(const LeafScanContext& c, std::uint32_t begin, std::uint32_t end,
                       BoundedHeap& heap, bool use_simd) {
    float distances[KdTree::kBlockSize];

    for (std::uint32_t base = begin; base < end; base += KdTree::kBlockSize) {
        const std::uint32_t n = std::min<std::uint32_t>(KdTree::kBlockSize, end - base);
        const float* bx = c.px + base;
        const float* by = c.py + base;
        const float* bz = c.pz + base;

        fill_block_distances(bx, by, bz, c.qx, c.qy, c.qz, distances, n, use_simd);

        if (heap.full()) {
            float block_min = distances[0];
            for (std::uint32_t j = 1; j < n; ++j) block_min = std::min(block_min, distances[j]);
            if (block_min >= heap.worst()) continue;
        }

        for (std::uint32_t j = 0; j < n; ++j) {
            if (distances[j] < heap.worst() || !heap.full()) {
                heap.offer(c.original[base + j], distances[j]);
            }
        }
    }
}
// LISTING_END scan_leaf_batched

void collect_leaf_scalar(const LeafScanContext& c, std::uint32_t begin, std::uint32_t end,
                         float radius2, std::vector<Neighbor>& out) {
    for (std::uint32_t i = begin; i < end; ++i) {
        const float dx = c.px[i] - c.qx;
        const float dy = c.py[i] - c.qy;
        const float dz = c.pz[i] - c.qz;
        const float d2 = dx * dx + dy * dy + dz * dz;
        if (d2 <= radius2) out.push_back({c.original[i], d2});
    }
}

void collect_leaf_batched(const LeafScanContext& c, std::uint32_t begin, std::uint32_t end,
                          float radius2, std::vector<Neighbor>& out, bool use_simd) {
    float distances[KdTree::kBlockSize];

    for (std::uint32_t base = begin; base < end; base += KdTree::kBlockSize) {
        const std::uint32_t n = std::min<std::uint32_t>(KdTree::kBlockSize, end - base);
        fill_block_distances(c.px + base, c.py + base, c.pz + base, c.qx, c.qy, c.qz, distances, n,
                             use_simd);
        for (std::uint32_t j = 0; j < n; ++j) {
            if (distances[j] <= radius2) out.push_back({c.original[base + j], distances[j]});
        }
    }
}

void dispatch_leaf_knn(const LeafScanContext& c, std::uint32_t begin, std::uint32_t end,
                       BoundedHeap& heap, NnPath path) {
    if (path == NnPath::Scalar) {
        scan_leaf_scalar(c, begin, end, heap);
    } else {
        scan_leaf_batched(c, begin, end, heap, path == NnPath::Simd);
    }
}

void dispatch_leaf_radius(const LeafScanContext& c, std::uint32_t begin, std::uint32_t end,
                          float radius2, std::vector<Neighbor>& out, NnPath path) {
    if (path == NnPath::Scalar) {
        collect_leaf_scalar(c, begin, end, radius2, out);
    } else {
        collect_leaf_batched(c, begin, end, radius2, out, path == NnPath::Simd);
    }
}

}  // namespace

// --- заявки -------------------------------------------------------------

namespace {

// Обхождането е итеративно, със собствен стек. Рекурсията стига дълбочина
// log2(n / leaf_size), тоест около 15 при милион точки, което е безопасно, но
// собственият стек държи и разстоянието до разделящата равнина и така
// отсичането става без повторно смятане.
struct StackEntry {
    std::uint32_t node;
    float axis_distance2;  // квадрат на разстоянието до равнината на предшественика
};

}  // namespace

void KdTree::knn(const Point3& query, std::size_t k, std::vector<Neighbor>& out, NnPath path) const {
    out.clear();
    if (px_.empty() || k == 0) return;

    k = std::min(k, px_.size());
    out.reserve(k);
    BoundedHeap heap{out, k};

    const LeafScanContext ctx{px_.data(), py_.data(), pz_.data(), original_index_.data(),
                              query.x,    query.y,    query.z};

    std::vector<StackEntry> stack;
    stack.reserve(64);
    stack.push_back({0U, 0.0F});

    while (!stack.empty()) {
        const StackEntry entry = stack.back();
        stack.pop_back();

        // Отсичане: подпространството не може да съдържа по-добър кандидат.
        if (heap.full() && entry.axis_distance2 >= heap.worst()) continue;

        std::uint32_t node_index = entry.node;
        while (true) {
            const Node& node = nodes_[node_index];
            if (node.axis < 0) {
                dispatch_leaf_knn(ctx, node.begin, node.end, heap, path);
                break;
            }

            const float q = (node.axis == 0) ? query.x : (node.axis == 1) ? query.y : query.z;
            const float diff = q - node.split;
            const std::uint32_t near_child = (diff < 0.0F) ? node_index + 1 : node.right_child;
            const std::uint32_t far_child = (diff < 0.0F) ? node.right_child : node_index + 1;

            const float far_distance2 = diff * diff;
            if (!heap.full() || far_distance2 < heap.worst()) {
                stack.push_back({far_child, far_distance2});
            }
            node_index = near_child;
        }
    }

    std::sort(out.begin(), out.end(), [](const Neighbor& a, const Neighbor& b) {
        return a.squared_distance < b.squared_distance;
    });
}

std::vector<Neighbor> KdTree::knn(const Point3& query, std::size_t k, NnPath path) const {
    std::vector<Neighbor> out;
    knn(query, k, out, path);
    return out;
}

bool KdTree::nearest(const Point3& query, Neighbor& out, NnPath path) const {
    if (px_.empty()) return false;

    float best = std::numeric_limits<float>::infinity();
    std::uint32_t best_index = 0;

    const float qx = query.x;
    const float qy = query.y;
    const float qz = query.z;

    std::vector<StackEntry> stack;
    stack.reserve(64);
    stack.push_back({0U, 0.0F});

    float distances[kBlockSize];

    while (!stack.empty()) {
        const StackEntry entry = stack.back();
        stack.pop_back();
        if (entry.axis_distance2 >= best) continue;

        std::uint32_t node_index = entry.node;
        while (true) {
            const Node& node = nodes_[node_index];
            if (node.axis < 0) {
                if (path == NnPath::Scalar) {
                    for (std::uint32_t i = node.begin; i < node.end; ++i) {
                        const float dx = px_[i] - qx;
                        const float dy = py_[i] - qy;
                        const float dz = pz_[i] - qz;
                        const float d2 = dx * dx + dy * dy + dz * dz;
                        if (d2 < best) {
                            best = d2;
                            best_index = original_index_[i];
                        }
                    }
                } else {
                    const bool use_simd = path == NnPath::Simd;
                    for (std::uint32_t base = node.begin; base < node.end; base += kBlockSize) {
                        const std::uint32_t n = std::min<std::uint32_t>(kBlockSize, node.end - base);
                        fill_block_distances(px_.data() + base, py_.data() + base, pz_.data() + base,
                                             qx, qy, qz, distances, n, use_simd);
                        for (std::uint32_t j = 0; j < n; ++j) {
                            if (distances[j] < best) {
                                best = distances[j];
                                best_index = original_index_[base + j];
                            }
                        }
                    }
                }
                break;
            }

            const float q = (node.axis == 0) ? qx : (node.axis == 1) ? qy : qz;
            const float diff = q - node.split;
            const std::uint32_t near_child = (diff < 0.0F) ? node_index + 1 : node.right_child;
            const std::uint32_t far_child = (diff < 0.0F) ? node.right_child : node_index + 1;

            const float far_distance2 = diff * diff;
            if (far_distance2 < best) stack.push_back({far_child, far_distance2});
            node_index = near_child;
        }
    }

    out.index = best_index;
    out.squared_distance = best;
    return true;
}

void KdTree::radius_search(const Point3& query, float radius, std::vector<Neighbor>& out,
                           NnPath path) const {
    out.clear();
    if (px_.empty() || radius < 0.0F) return;

    const float radius2 = radius * radius;
    const LeafScanContext ctx{px_.data(), py_.data(), pz_.data(), original_index_.data(),
                              query.x,    query.y,    query.z};

    std::vector<StackEntry> stack;
    stack.reserve(64);
    stack.push_back({0U, 0.0F});

    while (!stack.empty()) {
        const StackEntry entry = stack.back();
        stack.pop_back();
        if (entry.axis_distance2 > radius2) continue;

        std::uint32_t node_index = entry.node;
        while (true) {
            const Node& node = nodes_[node_index];
            if (node.axis < 0) {
                dispatch_leaf_radius(ctx, node.begin, node.end, radius2, out, path);
                break;
            }

            const float q = (node.axis == 0) ? query.x : (node.axis == 1) ? query.y : query.z;
            const float diff = q - node.split;
            const std::uint32_t near_child = (diff < 0.0F) ? node_index + 1 : node.right_child;
            const std::uint32_t far_child = (diff < 0.0F) ? node.right_child : node_index + 1;

            const float far_distance2 = diff * diff;
            if (far_distance2 <= radius2) stack.push_back({far_child, far_distance2});
            node_index = near_child;
        }
    }
}

std::vector<Neighbor> KdTree::radius_search(const Point3& query, float radius, NnPath path) const {
    std::vector<Neighbor> out;
    radius_search(query, radius, out, path);
    return out;
}

std::size_t KdTree::memory_bytes() const {
    return px_.capacity() * sizeof(float) * 3 +
           original_index_.capacity() * sizeof(std::uint32_t) + nodes_.capacity() * sizeof(Node);
}

std::vector<Neighbor> brute_force_knn(const PointCloud& cloud, const Point3& query, std::size_t k) {
    std::vector<Neighbor> all;
    all.reserve(cloud.size());
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        all.push_back({static_cast<std::uint32_t>(i), squared_distance(cloud.point(i), query)});
    }
    k = std::min(k, all.size());
    std::partial_sort(all.begin(), all.begin() + static_cast<std::ptrdiff_t>(k), all.end(),
                      [](const Neighbor& a, const Neighbor& b) {
                          return a.squared_distance < b.squared_distance;
                      });
    all.resize(k);
    return all;
}

std::vector<Neighbor> brute_force_radius(const PointCloud& cloud, const Point3& query, float radius) {
    const float radius2 = radius * radius;
    std::vector<Neighbor> out;
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        const float d2 = squared_distance(cloud.point(i), query);
        if (d2 <= radius2) out.push_back({static_cast<std::uint32_t>(i), d2});
    }
    return out;
}

}  // namespace pointforge
