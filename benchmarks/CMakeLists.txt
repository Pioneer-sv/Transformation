# 需要 Google Benchmark：sudo apt install libbenchmark-dev
find_package(benchmark CONFIG REQUIRED)

add_executable(benchmark_transformation benchmark_transformation.cpp)
target_link_libraries(benchmark_transformation
        PRIVATE transformation::transformation benchmark::benchmark_main transformation_warnings)

# Google Benchmark 的 API 随版本演进会产生新的 deprecated 警告，这类警告不当作错误
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(benchmark_transformation PRIVATE -Wno-error=deprecated-declarations)
endif()
