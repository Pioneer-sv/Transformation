# 需要 Google Benchmark：sudo apt install libbenchmark-dev
find_package(benchmark CONFIG REQUIRED)

add_executable(benchmark_transformation benchmark_transformation.cpp)
target_link_libraries(benchmark_transformation
        PRIVATE transformation::transformation benchmark::benchmark_main transformation_warnings)
