#include <iostream>
#include <omp.h>
#include <string>
#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "PointSet.hpp"

void pointSetToPolyscope(std::string name, PointSet *ps);
std::vector<glm::vec3> build_cube_from_minmax(glm::vec3 min, glm::vec3 max);
void generate_gaussian ();
void test_debug_readPly ();
void test_debug_subdivide ();
void test_debug_bounding_box(std::vector<glm::vec3> points);
void test_basic_polyscope ();
