#include <cmath>
#include <cstdio>
#include <fstream>
#include <numeric>
#include <string>

#include "pointforge/image_io.hpp"
#include "pointforge/image_ops.hpp"
#include "pointforge/labeling.hpp"
#include "pointforge/synthetic.hpp"
#include "test_framework.hpp"

using namespace pointforge;

namespace {

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

Image8 make_step_image(int width, int height, int step_x) {
    Image8 image(width, height, 1);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) image.at(x, y) = (x < step_x) ? 20 : 220;
    }
    return image;
}

}  // namespace

PF_TEST(image, pgm_round_trip_preserves_every_pixel) {
    const Image8 original = make_test_image(ImageSceneOptions{64, 48, 5.0F, 3U});
    TempFile file("pf_test_image.pgm");
    write_pgm(file.path, original);
    const Image8 read_back = read_pnm(file.path);

    PF_CHECK_EQ(read_back.width(), original.width());
    PF_CHECK_EQ(read_back.height(), original.height());
    PF_CHECK_EQ(read_back.channels(), 1);
    PF_CHECK(read_back.data() == original.data());
}

PF_TEST(image, ppm_round_trip_preserves_every_pixel) {
    Image8 colour(9, 7, 3);
    for (int y = 0; y < colour.height(); ++y) {
        for (int x = 0; x < colour.width(); ++x) {
            colour.at(x, y, 0) = static_cast<std::uint8_t>(x * 20);
            colour.at(x, y, 1) = static_cast<std::uint8_t>(y * 30);
            colour.at(x, y, 2) = static_cast<std::uint8_t>((x + y) * 10);
        }
    }
    TempFile file("pf_test_image.ppm");
    write_ppm(file.path, colour);
    const Image8 read_back = read_pnm(file.path);
    PF_CHECK_EQ(read_back.channels(), 3);
    PF_CHECK(read_back.data() == colour.data());

    // Един канал, записан като PPM, се повтаря по трите канала.
    const Image8 grey = make_test_image(ImageSceneOptions{8, 8, 0.0F, 1U});
    TempFile expanded("pf_test_expanded.ppm");
    write_ppm(expanded.path, grey);
    const Image8 back = read_pnm(expanded.path);
    PF_CHECK_EQ(back.channels(), 3);
    PF_CHECK_EQ(back.at(3, 4, 0), back.at(3, 4, 2));
    PF_CHECK_EQ(back.at(3, 4, 0), grey.at(3, 4));
}

PF_TEST(image, the_pnm_header_parser_handles_comments_and_scaling) {
    TempFile file("pf_test_comments.pgm");
    // Коментари между всички полета и максимална стойност, различна от 255.
    // Тялото се дописва отделно: нулевият байт в него не може да мине през
    // std::string, конструиран от литерал.
    {
        std::ofstream out(file.path, std::ios::binary);
        out << "P5\n"
               "# коментар веднага след вълшебната дума\n"
               "2 # коментар по средата на размерите\n"
               "2\n"
               "# коментар преди максимума\n"
               "1\n";
        const unsigned char body[] = {0, 1, 1, 0};
        out.write(reinterpret_cast<const char*>(body), sizeof(body));
    }

    const Image8 image = read_pnm(file.path);
    PF_CHECK_EQ(image.width(), 2);
    PF_CHECK_EQ(image.height(), 2);
    // Максимумът е 1, значи 1 се мащабира до 255, а 0 остава 0.
    PF_CHECK_EQ(static_cast<int>(image.at(0, 0)), 0);
    PF_CHECK_EQ(static_cast<int>(image.at(1, 0)), 255);
    PF_CHECK_EQ(static_cast<int>(image.at(0, 1)), 255);
}

PF_TEST(image, the_pnm_reader_rejects_malformed_input) {
    TempFile wrong_magic("pf_test_magic.pgm");
    write_text(wrong_magic.path, "P2\n2 2\n255\n0 0 0 0\n");
    PF_CHECK_THROWS(read_pnm(wrong_magic.path));

    // Заглавието обявява 16 пиксела, в тялото има 5 байта.
    TempFile truncated("pf_test_short.pgm");
    write_text(truncated.path, "P5\n4 4\n255\nshort");
    PF_CHECK_THROWS(read_pnm(truncated.path));

    PF_CHECK_THROWS(read_pnm("pf_test_no_such_image.pgm"));

    // Три канала не се записват като PGM: мълчаливото превръщане би скрило
    // грешка на извикващия.
    const Image8 colour(4, 4, 3);
    PF_CHECK_THROWS(write_pgm("pf_test_should_not_exist.pgm", colour));
}

PF_TEST(image, the_gaussian_kernel_is_normalised_and_symmetric) {
    for (const float sigma : {0.5F, 1.0F, 2.5F}) {
        const std::vector<float> kernel = gaussian_kernel(sigma);
        PF_CHECK(kernel.size() % 2 == 1);
        const double sum = std::accumulate(kernel.begin(), kernel.end(), 0.0);
        PF_CHECK_NEAR(sum, 1.0, 1e-6);
        for (std::size_t i = 0; i < kernel.size() / 2; ++i) {
            PF_CHECK_NEAR(kernel[i], kernel[kernel.size() - 1 - i], 1e-9);
        }
        // Върхът е в средата.
        PF_CHECK(kernel[kernel.size() / 2] >= kernel.front());
    }
    PF_CHECK_THROWS(gaussian_kernel(0.0F));
    PF_CHECK_THROWS(gaussian_kernel(-1.0F));
}

PF_TEST(image, blurring_a_constant_image_changes_nothing) {
    // Ако обработката на границата допълваше с нула, ръбовете щяха да
    // потъмнеят. Тази проверка е точно за това.
    ImageF flat(21, 17);
    for (float& v : flat.data()) v = 128.0F;

    const ImageF blurred = gaussian_blur(flat, 2.0F);
    for (int y = 0; y < blurred.height(); ++y) {
        for (int x = 0; x < blurred.width(); ++x) PF_CHECK_NEAR(blurred.at(x, y), 128.0, 1e-3);
    }
}

PF_TEST(image, blurring_lowers_the_contrast_of_a_step) {
    const ImageF step = to_float(make_step_image(40, 20, 20));
    const ImageF blurred = gaussian_blur(step, 2.0F);
    // Точно до ръба стойността вече не е крайната.
    PF_CHECK(blurred.at(19, 10) > 20.0F);
    PF_CHECK(blurred.at(20, 10) < 220.0F);
    // Далече от ръба стойностите остават.
    PF_CHECK_NEAR(blurred.at(2, 10), 20.0, 1.0);
    PF_CHECK_NEAR(blurred.at(37, 10), 220.0, 1.0);
}

PF_TEST(image, sobel_finds_a_vertical_edge_and_ignores_flat_regions) {
    const ImageF step = to_float(make_step_image(40, 24, 20));
    const Gradient gradient = sobel(step);

    // Вертикален контур: градиентът е по x, а по y е нула.
    PF_CHECK(std::fabs(gradient.gx.at(20, 12)) > 100.0F);
    PF_CHECK_NEAR(gradient.gy.at(20, 12), 0.0, 1e-3);
    // Далече от контура и двете компоненти са нула.
    PF_CHECK_NEAR(gradient.gx.at(5, 12), 0.0, 1e-3);
    PF_CHECK_NEAR(gradient.magnitude.at(5, 12), 0.0, 1e-3);
    // Големината е дължината на вектора.
    const float dx = gradient.gx.at(20, 12);
    const float dy = gradient.gy.at(20, 12);
    PF_CHECK_NEAR(gradient.magnitude.at(20, 12), std::sqrt(dx * dx + dy * dy), 1e-3);
}

PF_TEST(image, sobel_finds_a_horizontal_edge_on_the_other_axis) {
    Image8 image(24, 40, 1);
    for (int y = 0; y < image.height(); ++y) {
        for (int x = 0; x < image.width(); ++x) image.at(x, y) = (y < 20) ? 30 : 200;
    }
    const Gradient gradient = sobel(to_float(image));
    PF_CHECK(std::fabs(gradient.gy.at(12, 20)) > 100.0F);
    PF_CHECK_NEAR(gradient.gx.at(12, 20), 0.0, 1e-3);
}

PF_TEST(image, non_maximum_suppression_thins_a_thick_edge) {
    const ImageF blurred = gaussian_blur(to_float(make_step_image(60, 20, 30)), 2.0F);
    const Gradient gradient = sobel(blurred);
    const ImageF thin = non_maximum_suppression(gradient);

    // По един ред: изгладеният контур е широк, потиснатият е тесен.
    int wide = 0;
    int narrow = 0;
    for (int x = 0; x < 60; ++x) {
        if (gradient.magnitude.at(x, 10) > 1.0F) ++wide;
        if (thin.at(x, 10) > 1.0F) ++narrow;
    }
    PF_CHECK(wide > narrow);
    PF_CHECK(narrow >= 1);
    PF_CHECK(narrow <= 2);
    // Оцелелите стойности не са нови, а са взети от големината.
    for (int x = 0; x < 60; ++x) {
        if (thin.at(x, 10) > 0.0F) PF_CHECK_NEAR(thin.at(x, 10), gradient.magnitude.at(x, 10), 1e-6);
    }
}

PF_TEST(image, hysteresis_keeps_weak_pixels_only_when_connected) {
    ImageF response(9, 3);
    // Ред от слаби отговори, свързан със силен в единия край.
    for (int x = 0; x < 5; ++x) response.at(x, 1) = 30.0F;
    response.at(0, 1) = 90.0F;
    // Самотен слаб отговор, без връзка със силен.
    response.at(8, 1) = 30.0F;

    const Image8 edges = hysteresis_threshold(response, 20.0F, 60.0F);
    for (int x = 0; x < 5; ++x) PF_CHECK_EQ(static_cast<int>(edges.at(x, 1)), 255);
    PF_CHECK_EQ(static_cast<int>(edges.at(8, 1)), 0);
    PF_CHECK_EQ(static_cast<int>(edges.at(6, 1)), 0);
}

PF_TEST(image, connected_components_counts_separate_regions) {
    Image8 binary(20, 20, 1);
    // Три квадрата, които не се допират.
    for (int y = 1; y < 5; ++y) {
        for (int x = 1; x < 5; ++x) binary.at(x, y) = 255;
    }
    for (int y = 1; y < 4; ++y) {
        for (int x = 10; x < 18; ++x) binary.at(x, y) = 255;
    }
    for (int y = 12; y < 18; ++y) {
        for (int x = 12; x < 18; ++x) binary.at(x, y) = 255;
    }

    const LabelResult result = connected_components(binary);
    PF_CHECK_EQ(result.components.size(), std::size_t{3});

    std::uint32_t total = 0;
    for (const Component& component : result.components) total += component.area;
    PF_CHECK_EQ(total, std::uint32_t{16 + 24 + 36});

    // Номерата започват от 1 и вървят без прекъсване.
    for (std::size_t i = 0; i < result.components.size(); ++i) {
        PF_CHECK_EQ(result.components[i].label, static_cast<std::uint32_t>(i + 1));
    }
    // Ограждащ правоъгълник и център на тежестта на първия квадрат.
    PF_CHECK_EQ(result.components[0].min_x, 1);
    PF_CHECK_EQ(result.components[0].max_x, 4);
    PF_CHECK_NEAR(result.components[0].centroid_x, 2.5, 1e-9);
    PF_CHECK_NEAR(result.components[0].centroid_y, 2.5, 1e-9);
    // Фонът остава нула.
    PF_CHECK_EQ(result.label_at(0, 0), std::uint32_t{0});
}

PF_TEST(image, connectivity_decides_whether_a_diagonal_joins) {
    Image8 binary(6, 6, 1);
    binary.at(1, 1) = 255;
    binary.at(2, 2) = 255;

    PF_CHECK_EQ(connected_components(binary, Connectivity::Eight).components.size(), std::size_t{1});
    PF_CHECK_EQ(connected_components(binary, Connectivity::Four).components.size(), std::size_t{2});
}

PF_TEST(image, small_components_are_dropped_by_min_area) {
    Image8 binary(20, 20, 1);
    for (int y = 2; y < 8; ++y) {
        for (int x = 2; x < 8; ++x) binary.at(x, y) = 255;
    }
    binary.at(15, 15) = 255;  // единичен пиксел шум

    const LabelResult all = connected_components(binary, Connectivity::Eight, 1);
    PF_CHECK_EQ(all.components.size(), std::size_t{2});

    const LabelResult filtered = connected_components(binary, Connectivity::Eight, 4);
    PF_CHECK_EQ(filtered.components.size(), std::size_t{1});
    PF_CHECK_EQ(filtered.components[0].area, std::uint32_t{36});
    // Изхвърленият пиксел е върнат във фона.
    PF_CHECK_EQ(filtered.label_at(15, 15), std::uint32_t{0});
}

PF_TEST(image, the_full_pipeline_segments_the_generated_scene) {
    const Image8 scene = make_test_image(ImageSceneOptions{160, 120, 2.0F, 9U});
    const ImageF blurred = gaussian_blur(to_float(scene), 1.4F);
    const Image8 mask = threshold(blurred, 110.0F);
    const LabelResult result = connected_components(mask, Connectivity::Eight, 40);

    // Генераторът рисува три правоъгълника и два кръга. Единият кръг е с
    // яркост 130 и остава над прага, другият с 190 също.
    PF_CHECK_EQ(result.components.size(), std::size_t{5});
    for (const Component& component : result.components) {
        PF_CHECK(component.area >= 40);
        PF_CHECK(component.width() > 1);
        PF_CHECK(component.height() > 1);
    }
}

PF_TEST(image, conversions_between_the_two_raster_types) {
    Image8 colour(4, 4, 3);
    for (int y = 0; y < 4; ++y) {
        for (int x = 0; x < 4; ++x) {
            colour.at(x, y, 0) = 255;
            colour.at(x, y, 1) = 0;
            colour.at(x, y, 2) = 0;
        }
    }
    const Image8 grey = to_grayscale(colour);
    PF_CHECK_EQ(grey.channels(), 1);
    PF_CHECK_EQ(static_cast<int>(grey.at(0, 0)), 76);  // закръглено 0.299 * 255

    ImageF signed_values(3, 1);
    signed_values.at(0, 0) = -50.0F;
    signed_values.at(1, 0) = 0.0F;
    signed_values.at(2, 0) = 150.0F;

    const Image8 clipped = to_image8(signed_values);
    PF_CHECK_EQ(static_cast<int>(clipped.at(0, 0)), 0);
    PF_CHECK_EQ(static_cast<int>(clipped.at(2, 0)), 150);

    const Image8 scaled = normalize_to_image8(signed_values);
    PF_CHECK_EQ(static_cast<int>(scaled.at(0, 0)), 0);
    PF_CHECK_EQ(static_cast<int>(scaled.at(2, 0)), 255);
    PF_CHECK(scaled.at(1, 0) > 0 && scaled.at(1, 0) < 255);

    // Постоянно изображение: мащабирането не бива да дели на нула.
    ImageF flat(2, 2);
    const Image8 flat_scaled = normalize_to_image8(flat);
    PF_CHECK_EQ(static_cast<int>(flat_scaled.at(0, 0)), 0);
}
