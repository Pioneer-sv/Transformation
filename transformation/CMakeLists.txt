# 指定Cmake的最小版本为3.22
cmake_minimum_required(VERSION 3.22)

# 指定项目名为transformation
project(transformation)

# 显示PROJECT_SOURCE_DIR宏定义信息
message(STATUS "项目根目录: ${PROJECT_SOURCE_DIR}")

# 查找Eigen3库
find_package(Eigen3 REQUIRED)

# 指定可执行文件存放目录
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# 根据源文件生成可执行文件
add_executable(transformation ${PROJECT_SOURCE_DIR}/src/main.cpp)

# 链接include目录下的头文件
target_include_directories(transformation PRIVATE ${PROJECT_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIRS})
