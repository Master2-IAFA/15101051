#pragma once

#include <iostream>
#include <omp.h>
#include <string>
#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "polyscope/curve_network.h"
#include "utils.t.hpp"
#include "PointSet.t.hpp"
#include "Octree/BaseOctree.t.hpp"
#include "Octree/Octree.t.hpp"
#include "Octree/InputOctree.t.hpp"
#include "AlgebraicSphere.t.hpp"
#include "kernels.t.hpp"

template<class VecType, class PointType>
polyscope::PointCloud* pointSetToPolyscope(std::string name, PointSet<PointType> *ps);

/** create a hypercube (square/cube) from its diagonal positions
 * @param min first diagonal coordinate
 * @param max second diagonal coordinate
 * @return hypercube coordinates (4/8 coordinates)
 */
template< class VecType >
std::vector<VecType> build_cube_from_minmax(VecType min, VecType max);

/** generate a .ply file containing a sampeled 3D gaussian*/
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
 * @brief Implementation of the projection methode from the MLoD paper.
 * @param octree : Root octree of the input point cloud
 * @param q : Point q that we want to project into the octree
 */
template<typename statistics, typename VecType, typename point, typename PointType>
AlgebraicSphere<VecType, statistics> projection (InputOctree<VecType, statistics, PointType>* octree, float (*kernel)(VecType& ,VecType& ) ,VecType& q);

/**
 *
 * @brief This function draws the nodes traversed for 1 random point.
 * MOREOVER : It creates a random point into the bounding box, and try to project it into the octree, in order to create the new projected point.
 * MOREOVER : It takes all of the point cloud and try to project all of its points into the octree, and creates a new point cloud (the projected one).
 *
 */
template< class VecType, class StatType, class PointType >
polyscope::PointCloud * draw_traverseOctree_onePoint(InputOctree<VecType, StatType, PointType> * oct, PointSet<point3d> *ps) ;

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
void display_sphere( std::string name, glm::vec3 center, float radius) ;

template<typename Data, typename VecType, typename OctreeType>
polyscope::CurveNetwork* drawOctree(std::string name, std::vector<BaseOctree<Data, VecType, OctreeType>*> octree);

/**
 * @brief function to vizualise points sliding accross their translation
 */
void slide_points(polyscope::PointCloud *, polyscope::PointCloud * pc_final, int nb_slider_max, int nb_slider) ;


/**
 * @author Léo 
 * 
 * @brief This function take an octree, a depth and an idx for the node at the given depth and show its algebraic sphere. 
 * @param name name of the displayed sphere.
 * @param oct 
 * @param depth Depth of the node we want
 * @param num_child Num of the node for the given depth
 */
template< class VecType, class StatType, class PointType >
void node_stats_to_sphere ( std::string name, InputOctree<VecType, StatType, PointType> * oct, int depth, int num_child);

/**
 * @author Léo 
 * 
 * @brief This function takes a point and its fitted algebraic sphere to display the point, the sphere and its projection. 
 * @param name name of the displayed sphere.
 * @param point point that we want to fit a sphere.
 * @param sphere fitted sphere.
 */
template< class VecType, class StatType >
void point_and_stats_to_sphere ( std::string point_name, std::string name, VecType point, VecType end, AlgebraicSphere<VecType, StatType> sphere);
