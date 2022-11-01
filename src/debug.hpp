#pragma once
#include <iostream>
#include <omp.h>
#include <string>
#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "polyscope/curve_network.h"
#include "PointSet.hpp"
#include "utils.hpp"
#include "Octree.hpp"


void pointSetToPolyscope(std::string name, PointSet *ps);

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


/**
 * @brief displays sphere on polyscope according to its radius and center
 */
void display_sphere(float radius, glm::vec3 center) ;

/**
 * @brief function to fit an algebraic sphere to projected point according to the node's statistics
 */
void fit_sphere_on_node(InputOctree * octree, PointSet * ps, glm::vec3 q);

/**
 * @brief function display only points in cube
 */
vector<glm::vec3> delete_points_not_in_cube(InputOctree * octree, PointSet * ps);

/**
 * @brief function to compute center from alebraic sphere parameters
 */
glm::vec3 get_center(glm::vec3 m_ul, float m_uq);

/**
 * @brief function to compute radius from algebraic sphere parameters
 */
float get_radius(glm::vec3 m_ul, float m_uc, float m_uq);
