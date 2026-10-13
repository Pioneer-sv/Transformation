cmake_minimum_required(VERSION 3.22)

project(Transformation
        VERSION 2.2.0
        DESCRIPTION "基于 Eigen 的旋转表示转换库（旋转矩阵 / 欧拉角 / 轴角 / 四元数）"
        LANGUAGES CXX)

# ---------------------------------------------------------------- 头文件库
add_library(transformation INTERFACE)
add_library(transformation::transformation ALIAS transformation)

target_include_directories(transformation INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_compile_features(transformation INTERFACE cxx_std_17)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries(transformation INTERFACE Eigen3::Eigen)

# ---------------------------------------------------------------- 编译警告
option(TRANSFORMATION_WARNINGS_AS_ERRORS "把编译警告当作错误（CI 中开启）" OFF)

add_library(transformation_warnings INTERFACE)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(transformation_warnings INTERFACE -Wall -Wextra -Wpedantic)
    if(TRANSFORMATION_WARNINGS_AS_ERRORS)
        target_compile_options(transformation_warnings INTERFACE -Werror)
    endif()
endif()

# ---------------------------------------------------------------- 覆盖率
if(TRANSFORMATION_ENABLE_COVERAGE)
    if(NOT CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        message(WARNING "覆盖率统计仅支持 GCC/Clang")
    else()
        add_compile_options(--coverage -O0 -g)
        add_link_options(--coverage)
        message(STATUS "已开启覆盖率统计（--coverage）")
    endif()
endif()

# ---------------------------------------------------------------- 可选目标
option(TRANSFORMATION_BUILD_TESTS "构建测试" ON)
option(TRANSFORMATION_BUILD_EXAMPLES "构建示例程序" ON)
option(TRANSFORMATION_BUILD_BENCHMARKS "构建性能基准（需要 Google Benchmark）" OFF)
option(TRANSFORMATION_ENABLE_COVERAGE "开启覆盖率统计（GCC/Clang 的 --coverage）" OFF)

if(TRANSFORMATION_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

if(TRANSFORMATION_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

if(TRANSFORMATION_BUILD_BENCHMARKS)
    add_subdirectory(benchmarks)
endif()

# ---------------------------------------------------------------- 安装
include(GNUInstallDirs)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

message(STATUS "transformation ${PROJECT_VERSION}：Eigen3 ${Eigen3_VERSION}，头文件目录 ${CMAKE_CURRENT_SOURCE_DIR}/include")
