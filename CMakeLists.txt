cmake_minimum_required(VERSION 2.10)
project(Avantis3D)

include(cmake/Modules/conan.cmake)
conan_cmake_run(
  CONANFILE conanfile.txt
  BASIC_SETUP CMAKE_TARGETS
  BUILD missing)

add_executable(triangle_cross
  src/equal_to_zero.h
  src/fmap.h
  src/fmap.cpp
  src/line_segment_line_segment_intersection_2d.cpp
  src/line_segment_line_segment_intersection_3d.cpp
  src/triangle_line_segment_intersection_2d.cpp
  src/triangle_line_segment_intersection_3d.cpp
  src/types.h
  src/main.cpp)

set_target_properties(triangle_cross PROPERTIES
  CXX_STANDARD 17
  CXX_EXTENSIONS OFF)

target_link_libraries(triangle_cross
  CONAN_PKG::blaze
  CONAN_PKG::boost
  CONAN_PKG::lapack
  CONAN_PKG::gtest)

enable_testing()

add_test(
  NAME triangle_cross_test
  COMMAND triangle_cross)
