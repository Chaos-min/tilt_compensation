cmake_minimum_required(VERSION 3.16)
project(tilt_compensation LANGUAGES CXX)

add_library(tilt_compensation src/tilt_compensation.cpp)
target_include_directories(tilt_compensation PUBLIC include)
target_compile_features(tilt_compensation PUBLIC cxx_std_11)

add_executable(tilt_compensation_test test/test.cpp)
target_link_libraries(tilt_compensation_test PRIVATE tilt_compensation)

enable_testing()
add_test(NAME tilt_compensation_test COMMAND tilt_compensation_test)

# Add a test for the tilt_compensation library
add_executable(tilt_compensation_example test/e_cpp.cpp)
target_link_libraries(tilt_compensation_example PRIVATE tilt_compensation)

add_executable(tilt_compensation_file_example test/e_tilt.cpp)
target_link_libraries(tilt_compensation_file_example PRIVATE tilt_compensation)
