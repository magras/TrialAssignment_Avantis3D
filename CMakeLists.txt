cmake_minimum_required(VERSION 2.10)
project(Avantis3D)

include(cmake/Modules/conan.cmake)
conan_cmake_run(CONANFILE conanfile.txt)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

add_executable(trangle_cross src/main.cpp)

set_target_properties(trangle_cross PROPERTIES
  CXX_STANDARD 17
  CXX_EXTENSIONS OFF)

target_link_libraries(trangle_cross ${CONAN_LIBS})

add_custom_command(TARGET trangle_cross POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${CONAN_BIN_DIRS_LAPACK}"
    $<TARGET_FILE_DIR:trangle_cross>)
