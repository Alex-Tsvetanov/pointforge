cmake_minimum_required(VERSION 3.20)

project(PointForge
        VERSION 0.1.0
        DESCRIPTION "Point cloud and depth image processing with an accelerated spatial index"
        LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Изграждане с C++20 и CMake. Единствената външна зависимост е xsimd и е
# вградена в third_party/xsimd (само заглавки). Няма FetchContent и няма
# мрежов достъп при конфигуриране.
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    message(STATUS "PointForge: CMAKE_BUILD_TYPE не е зададен, приема се Release")
endif()

option(POINTFORGE_NATIVE_ARCH
       "Компилира с -march=native. Изключено по подразбиране: изгражданият файл спира да е преносим."
       OFF)
option(POINTFORGE_WARNINGS_AS_ERRORS "Предупрежденията се третират като грешки" OFF)

add_library(pointforge_flags INTERFACE)
if(MSVC)
    target_compile_options(pointforge_flags INTERFACE /W4 /permissive-)
    if(POINTFORGE_WARNINGS_AS_ERRORS)
        target_compile_options(pointforge_flags INTERFACE /WX)
    endif()
else()
    target_compile_options(pointforge_flags INTERFACE -Wall -Wextra)
    if(POINTFORGE_WARNINGS_AS_ERRORS)
        target_compile_options(pointforge_flags INTERFACE -Werror)
    endif()
    if(POINTFORGE_NATIVE_ARCH)
        target_compile_options(pointforge_flags INTERFACE -march=native)
    endif()
    # -O3 за Release: пакетният път е написан за векторизация; при -O2 GCC често
    # не разгъва блок от константна дължина. Флагът е само за Release.
    target_compile_options(pointforge_flags INTERFACE $<$<CONFIG:Release>:-O3>)
endif()

# Под MinGW изпълнимите файлове се свързват статично със стандартната
# библиотека. Причината е наблюдавана, не предпазна: на машина с няколко
# инсталации на MinGW (например тази на Git for Windows) програмата зарежда
# първата libstdc++-6.dll от PATH, а тя може да е по-стара от компилатора,
# с който е компилиран проектът. Резултатът е падане още при първото
# отваряне на файл, което няма нищо общо с кода.
if(MINGW)
    target_link_options(pointforge_flags INTERFACE -static-libstdc++ -static-libgcc)
endif()

# xsimd: header-only, вендорирана в third_party/. Не се извлича при configure.
add_library(pointforge_xsimd INTERFACE)
target_include_directories(pointforge_xsimd INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/xsimd/include>)

add_library(pointforge
    src/transform.cpp
    src/cloud_io.cpp
    src/kdtree.cpp
    src/voxel_grid.cpp
    src/icp.cpp
    src/image.cpp
    src/image_io.cpp
    src/image_ops.cpp
    src/labeling.cpp
    src/synthetic.cpp
)
target_include_directories(pointforge PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(pointforge PRIVATE pointforge_flags pointforge_xsimd)
add_library(PointForge::pointforge ALIAS pointforge)

# ---------------------------------------------------------------------------
# Демонстрация
# ---------------------------------------------------------------------------
add_executable(pointforge_demo apps/demo.cpp)
target_link_libraries(pointforge_demo PRIVATE pointforge pointforge_flags)

# Цел `demo`: изгражда и пуска демонстрацията с една команда.
add_custom_target(demo
    COMMAND pointforge_demo --output-dir "${CMAKE_CURRENT_BINARY_DIR}/demo-output"
    DEPENDS pointforge_demo
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    COMMENT "PointForge: пускане на демонстрацията"
    USES_TERMINAL)

# ---------------------------------------------------------------------------
# Измервания
# ---------------------------------------------------------------------------
add_executable(pointforge_bench benchmarks/bench.cpp)
target_link_libraries(pointforge_bench PRIVATE pointforge pointforge_flags)

add_custom_target(bench
    COMMAND pointforge_bench
    DEPENDS pointforge_bench
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    COMMENT "PointForge: пускане на измерванията"
    USES_TERMINAL)

# ---------------------------------------------------------------------------
# Проверки
# ---------------------------------------------------------------------------
include(CTest)
enable_testing()

add_executable(pointforge_tests
    tests/test_main.cpp
    tests/test_geometry.cpp
    tests/test_cloud_io.cpp
    tests/test_kdtree.cpp
    tests/test_voxel_grid.cpp
    tests/test_icp.cpp
    tests/test_image.cpp
)
target_include_directories(pointforge_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_link_libraries(pointforge_tests PRIVATE pointforge pointforge_flags)

# По един запис в CTest на група проверки. Групите отговарят на файловете в
# tests/, а самата програма поема името на групата като избор.
set(POINTFORGE_TEST_SUITES geometry cloud_io kdtree voxel_grid icp image)
foreach(suite IN LISTS POINTFORGE_TEST_SUITES)
    add_test(NAME ${suite} COMMAND pointforge_tests ${suite})
    set_tests_properties(${suite} PROPERTIES
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
endforeach()
