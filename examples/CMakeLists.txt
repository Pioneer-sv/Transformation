add_executable(demo demo.cpp)
target_link_libraries(demo PRIVATE transformation::transformation transformation_warnings)
