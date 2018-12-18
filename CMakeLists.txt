cmake_minimum_required(VERSION 2.10)
project(Avantis3D)

include(cmake/Modules/conan.cmake)
conan_cmake_run(CONANFILE conanfile.txt)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

add_executable(triangle_cross src/main.cpp)

set_target_properties(triangle_cross PROPERTIES
  CXX_STANDARD 17
  CXX_EXTENSIONS OFF)

target_link_libraries(triangle_cross ${CONAN_LIBS})

add_custom_command(TARGET triangle_cross POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${CONAN_BIN_DIRS_LAPACK}"
    $<TARGET_FILE_DIR:triangle_cross>)

enable_testing()

add_test(triangle_cross_test
  triangle_cross
  )
