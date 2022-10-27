project("Point_cloud")

cmake_minimum_required(VERSION 2.8)
cmake_policy(SET CMP0043 NEW)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/external/polyscope")

FILE(GLOB_RECURSE folder_source ${CMAKE_SOURCE_DIR}/src/*.cpp)
FILE(GLOB_RECURSE folder_header ${CMAKE_SOURCE_DIR}/src/*.hpp)

add_executable(PointCloud ${folder_source} ${folder_header})

target_link_libraries(PointCloud polyscope)
