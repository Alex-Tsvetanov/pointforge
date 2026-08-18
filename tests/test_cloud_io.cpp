#include <cstdio>
#include <fstream>
#include <string>

#include "pointforge/cloud_io.hpp"
#include "pointforge/synthetic.hpp"
#include "test_framework.hpp"

using namespace pointforge;

namespace {

// Временните файлове се пишат в текущата директория, която CTest поставя в
// дървото на изграждането, и се изтриват след проверката.
struct TempFile {
    std::string path;
    explicit TempFile(const std::string& name) : path(name) {}
    ~TempFile() { std::remove(path.c_str()); }
    TempFile(const TempFile&) = delete;
    TempFile& operator=(const TempFile&) = delete;
};

void write_text(const std::string& path, const std::string& content) {
    std::ofstream file(path, std::ios::binary);
    file << content;
}

void check_clouds_match(const PointCloud& a, const PointCloud& b, float tolerance) {
    PF_CHECK_EQ(a.size(), b.size());
    if (a.size() != b.size()) return;
    for (std::size_t i = 0; i < a.size(); ++i) {
        PF_CHECK_NEAR(a.point(i).x, b.point(i).x, tolerance);
        PF_CHECK_NEAR(a.point(i).y, b.point(i).y, tolerance);
        PF_CHECK_NEAR(a.point(i).z, b.point(i).z, tolerance);
    }
}

}  // namespace

PF_TEST(cloud_io, ply_round_trip_preserves_the_cloud) {
    const PointCloud original = make_scene(SceneOptions{200, 150, 150});
    TempFile file("pf_test_roundtrip.ply");
    write_ply(file.path, original);
    check_clouds_match(original, read_ply(file.path), 1e-5F);
}

PF_TEST(cloud_io, pcd_round_trip_preserves_the_cloud) {
    const PointCloud original = make_scene(SceneOptions{120, 90, 90});
    TempFile file("pf_test_roundtrip.pcd");
    write_pcd(file.path, original);
    check_clouds_match(original, read_pcd(file.path), 1e-5F);
}

PF_TEST(cloud_io, extension_dispatch_picks_the_right_reader) {
    PointCloud cloud;
    cloud.push_back(1.0F, 2.0F, 3.0F);
    cloud.push_back(-4.5F, 0.25F, 7.75F);

    TempFile ply("pf_test_dispatch.ply");
    TempFile pcd("pf_test_dispatch.pcd");
    write_cloud(ply.path, cloud);
    write_cloud(pcd.path, cloud);
    check_clouds_match(cloud, read_cloud(ply.path), 1e-6F);
    check_clouds_match(cloud, read_cloud(pcd.path), 1e-6F);

    PF_CHECK_THROWS(read_cloud("pf_test_dispatch.xyz"));
}

PF_TEST(cloud_io, ply_reader_selects_columns_by_name) {
    // Полетата са в неочакван ред и има допълнителни: разборът трябва да ги
    // намери по име, а не по позиция.
    TempFile file("pf_test_columns.ply");
    write_text(file.path,
               "ply\n"
               "format ascii 1.0\n"
               "comment произволен коментар\n"
               "element vertex 2\n"
               "property float z\n"
               "property uchar intensity\n"
               "property float x\n"
               "property float y\n"
               "end_header\n"
               "30 7 10 20\n"
               "60 9 40 50\n");

    const PointCloud cloud = read_ply(file.path);
    PF_CHECK_EQ(cloud.size(), std::size_t{2});
    PF_CHECK_NEAR(cloud.point(0).x, 10.0, 1e-6);
    PF_CHECK_NEAR(cloud.point(0).y, 20.0, 1e-6);
    PF_CHECK_NEAR(cloud.point(0).z, 30.0, 1e-6);
    PF_CHECK_NEAR(cloud.point(1).x, 40.0, 1e-6);
    PF_CHECK_NEAR(cloud.point(1).z, 60.0, 1e-6);
}

PF_TEST(cloud_io, ply_reader_rejects_malformed_input) {
    TempFile binary("pf_test_binary.ply");
    write_text(binary.path,
               "ply\n"
               "format binary_little_endian 1.0\n"
               "element vertex 1\n"
               "property float x\n"
               "property float y\n"
               "property float z\n"
               "end_header\n");
    PF_CHECK_THROWS(read_ply(binary.path));

    // Заглавието обявява три точки, а тялото съдържа две.
    TempFile truncated("pf_test_truncated.ply");
    write_text(truncated.path,
               "ply\n"
               "format ascii 1.0\n"
               "element vertex 3\n"
               "property float x\n"
               "property float y\n"
               "property float z\n"
               "end_header\n"
               "0 0 0\n"
               "1 1 1\n");
    PF_CHECK_THROWS(read_ply(truncated.path));

    TempFile missing_z("pf_test_missing_z.ply");
    write_text(missing_z.path,
               "ply\n"
               "format ascii 1.0\n"
               "element vertex 1\n"
               "property float x\n"
               "property float y\n"
               "end_header\n"
               "0 0\n");
    PF_CHECK_THROWS(read_ply(missing_z.path));

    PF_CHECK_THROWS(read_ply("pf_test_no_such_file.ply"));
}

PF_TEST(cloud_io, pcd_reader_selects_columns_by_name) {
    TempFile file("pf_test_fields.pcd");
    write_text(file.path,
               "# .PCD v0.7 - Point Cloud Data file format\n"
               "VERSION 0.7\n"
               "FIELDS x y z rgb\n"
               "SIZE 4 4 4 4\n"
               "TYPE F F F F\n"
               "COUNT 1 1 1 1\n"
               "WIDTH 2\n"
               "HEIGHT 1\n"
               "VIEWPOINT 0 0 0 1 0 0 0\n"
               "POINTS 2\n"
               "DATA ascii\n"
               "1 2 3 100\n"
               "4 5 6 200\n");

    const PointCloud cloud = read_pcd(file.path);
    PF_CHECK_EQ(cloud.size(), std::size_t{2});
    PF_CHECK_NEAR(cloud.point(1).x, 4.0, 1e-6);
    PF_CHECK_NEAR(cloud.point(1).y, 5.0, 1e-6);
    PF_CHECK_NEAR(cloud.point(1).z, 6.0, 1e-6);
}

PF_TEST(cloud_io, pcd_reader_rejects_binary_data) {
    TempFile file("pf_test_binary.pcd");
    write_text(file.path,
               "VERSION 0.7\n"
               "FIELDS x y z\n"
               "SIZE 4 4 4\n"
               "TYPE F F F\n"
               "COUNT 1 1 1\n"
               "WIDTH 1\n"
               "HEIGHT 1\n"
               "POINTS 1\n"
               "DATA binary\n");
    PF_CHECK_THROWS(read_pcd(file.path));
}

PF_TEST(cloud_io, empty_cloud_round_trips) {
    const PointCloud empty;
    TempFile file("pf_test_empty.ply");
    write_ply(file.path, empty);
    const PointCloud read_back = read_ply(file.path);
    PF_CHECK(read_back.empty());
}
