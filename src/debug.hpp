#pragma once

#include "utils.t.hpp"

#include <iostream>
#include <omp.h>
#include <string>

#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "polyscope/curve_network.h"

/** converts ps pointset to a polyscope point cloud and draws it
 * @param name pointset displayed name in polyscope
 * @param ps point set (3d)
 */
void pointSetToPolyscope(std::string name, PointSet<point3d> *ps);

/** converts ps pointset to a polyscope point cloud and draws it
 * @param name pointset displayed name in polyscope
 * @param ps point set (2d)
 */
void pointSet2dToPolyscope (std::string name, PointSet<point2d> *ps);

/** given min and max of a cube, returns vector of each point of the cube
 * @param min : min point of cube
 * @param max : max point of cube
 */
std::vector<glm::vec3> build_cube_from_minmax(glm::vec3 min, glm::vec3 max);

/**  generates small gaussian point cloud
 */
void generate_gaussian ();

/**  reads gaussian from ply file and displays it on polyscope
 */
void test_debug_readPly ();

/**  subdivides an octree
 */
void test_debug_subdivide ();

void test_debug_bounding_box(std::vector<glm::vec3> points);

void test_basic_polyscope ();

/**  given min and max of cube, draws cube on polyscope
 * @param name : name of polyscope window
 * @param min : min point of cube
 * @param max : max point of cube
 */
void drawCube(std::string name, glm::vec3 min, glm::vec3 max);

/**  given min and max of a 2d square, draws square on polyscope
 * @param name : name of polyscope window
 * @param min : min point of cube
 * @param max : max point of cube
 */
void drawSquare(std::string name, glm::vec2 min, glm::vec2 max);

/**  given min and max of a 2d square, returns vector of each point of the square
 * @param min : min point of cube
 * @param max : max point of cube
 */
std::vector<glm::vec2> build_square_from_minmax(glm::vec2 min, glm::vec2 max);

void draw_diagonal(std::string name, glm::vec3 min, glm::vec3 max);

/** compiles all cells of an octree into one polyscope drawable curve network and returns it
 * @param name octree dislayed name in polyscope
 * @param octree target
 * @return 3d curve network representing entry octree
 */
polyscope::CurveNetwork* drawOctree(std::string name, std::vector<Octree<statistics3d, glm::vec3> *> octree);

/** compiles all cells of a quadtree into one polyscope drawable curve network and returns it
 * @param name quadtree dislayed name in polyscope
 * @param octree target quadtree
 * @return 2d curve network representing entry quadtree
 */
polyscope::CurveNetwork* drawQuadtree (std::string name, std::vector<Octree<statistics2d, glm::vec2> *> quad);

PointSet<point2d> generate2dGaussian ();
