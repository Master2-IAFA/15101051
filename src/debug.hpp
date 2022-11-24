#pragma once

#include <iostream>
#include <omp.h>
#include <string>
#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "polyscope/curve_network.h"
#include "utils.t.hpp"
// #include "blending.t.hpp"

polyscope::PointCloud * pointSetToPolyscope(std::string name, PointSet<point3d> *ps);
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
 * @brief reads gaussian from ply file and displays it on polyscope
 */
void test_debug_subdivide ();



void test_debug_bounding_box(std::vector<glm::vec3> points);
/**
 *
 * @brief This function draws the nodes traversed for 1 random point.
 * MOREOVER : It creates a random point into the bounding box, and try to project it into the octree, in order to create the new projected point.
 * MOREOVER : It takes all of the point cloud and try to project all of its points into the octree, and creates a new point cloud (the projected one).
 *
 */
polyscope::PointCloud * draw_traverseOctree_onePoint (Octree<statistics3d, glm::vec3> *oct, PointSet<point3d> *ps);


void test_basic_polyscope ();

/**
 * @brief given min and max of cube, draws cube on polyscope
 * @param name : name of polyscope window
 * @param min : min point of cube
 * @param max : max point of cube
 */
void drawCube(std::string name, glm::vec3 min, glm::vec3 max);

void draw_diagonal(std::string name, glm::vec3 min, glm::vec3 max);

/**
 * @brief displays sphere on polyscope according to its radius and center
 */
void display_sphere(glm::vec3 center, float radius) ;

polyscope::CurveNetwork* drawOctree(std::string name, std::vector<Octree<statistics3d, glm::vec3> *> octree);

/**
 * @brief function to vizualise points sliding accross their translation
 */
void slide_points(polyscope::PointCloud *, polyscope::PointCloud * pc_final, int nb_slider_max, int nb_slider) ;
