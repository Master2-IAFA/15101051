#pragma once

#include <iostream>
#include <omp.h>
#include <string>
#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "polyscope/curve_network.h"
#include "utils.t.hpp"

void pointSetToPolyscope(std::string name, PointSet<point3d> *ps);
void pointSet2dToPolyscope (std::string name, PointSet<point2d> *ps);

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

void drawSquare(std::string name, glm::vec2 min, glm::vec2 max) ;

std::vector<glm::vec2> build_square_from_minmax(glm::vec2 min, glm::vec2 max);

void draw_diagonal(std::string name, glm::vec3 min, glm::vec3 max);

polyscope::CurveNetwork* drawOctree(std::string name, std::vector<Octree<statistics3d, glm::vec3> *> octree);

polyscope::CurveNetwork* drawQuadtree (std::string name, std::vector<Octree<statistics2d, glm::vec2> *> octree);

PointSet<point2d> generate2dGaussian ();
