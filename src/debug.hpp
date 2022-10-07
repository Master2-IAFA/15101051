#pragma once
#include <iostream>
#include <omp.h>
#include <string>
#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "polyscope/curve_network.h"
#include "PointSet.hpp"

void pointSetToPolyscope(std::string name, PointSet *ps);

std::vector<glm::vec3> build_cube_from_minmax(glm::vec3 min, glm::vec3 max);

void drawCube(glm::vec3 min, glm::vec3 max);
