project("Point_cloud")

cmake_minimum_required(VERSION 2.8)
cmake_policy(SET CMP0043 NEW)

set(default_build_type "Release")
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

include_directories(external)
set(open_mesh_sources
        external/OpenMesh
        external/OpenMesh/Core
        external/OpenMesh/Core/Geometry
        external/OpenMesh/Core/IO
        external/OpenMesh/Core/IO/exporter
        external/OpenMesh/Core/IO/importer
        external/OpenMesh/Core/IO/reader
        external/OpenMesh/Core/IO/writer
        external/OpenMesh/Core/Mesh
        external/OpenMesh/Core/Mesh/gen
        external/OpenMesh/Core/System
        external/OpenMesh/Core/Templates
        external/OpenMesh/Core/Utils
        )

# Add headers recursively to headers
file(GLOB_RECURSE om_headers "*.h*" ${open_mesh_sources})
# Add sources recursively to sources
file(GLOB_RECURSE om_sources "*.cc" ${open_mesh_sources})
# We need a static build of Open mesh
add_definitions(-DOM_STATIC_BUILD)

find_package(Doxygen)
find_package(OpenMP)

if (DOXYGEN_FOUND)
    # set input and output files
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    # request to configure the file
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    message("Doxygen build started")

    # note the option ALL which allows to build the docs together with the application
    add_custom_target( doc_doxygen ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/doc
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM )
else (DOXYGEN_FOUND)
  message("Doxygen need to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/external/polyscope")


FILE(GLOB_RECURSE folder_source ${CMAKE_SOURCE_DIR}/src/*.cpp)
FILE(GLOB_RECURSE folder_header ${CMAKE_SOURCE_DIR}/src/*.hpp)

add_executable(PointCloud ${folder_source} ${om_sources} ${folder_header} ${om_headers})

target_link_libraries(PointCloud polyscope OpenMP::OpenMP_CXX)
