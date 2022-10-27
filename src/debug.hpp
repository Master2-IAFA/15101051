#pragma once
#include <iostream>
#include <omp.h>
#include <string>
#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "polyscope/curve_network.h"
#include "PointSet.hpp"
#include "utils.hpp"

void pointSetToPolyscope(std::string name, PointSet *ps);
void pointSet2dToPolyscope (std::string name, PointSet *ps);

/**
 * @brief given min and max of a cube, returns vector of each point of the cube
 * @param min : min point of cube
 * @param max : max point of cube
 */
std::vector<glm::vec3> build_cube_from_minmax(glm::vec3 min, glm::vec3 max);

/**
 * @brief generates small gaussian point cloud
 */
void generate_gaussian ();

/**
 * @brief reads gaussian from ply file and displays it on polyscope
 */
void test_debug_readPly ();

/**
 * @brief subdivides an octree
 */
void test_debug_subdivide ();


void test_debug_bounding_box(std::vector<glm::vec3> points);
void test_basic_polyscope ();

/**
 * @brief given min and max of cube, draws cube on polyscope
 * @param name : name of polyscope window
 * @param min : min point of cube
 * @param max : max point of cube
 */
void drawCube(std::string name, glm::vec3 min, glm::vec3 max);

void draw_diagonal(std::string name, glm::vec3 min, glm::vec3 max);

polyscope::CurveNetwork* drawOctree(std::string name, std::vector<InputOctree *> octree);

PointSet generate2dGaussian ();
