#pragma once

#include <iostream>
#include <glm/glm.hpp>
#include "utils.t.hpp"

// This parameter LAMBDA is used as a factor to multiply the radius of the bounding sphere, in order to obtain the protection sphere.
#define LAMBDA 1.0f

/**
 * @brief Kernel used into the projection process it takes 2 points and return a scalar
 * @param p : A point that will be compared with q
 * @param q : A point that will be compared with p
 */
template<typename VecType>
float gaussian_mixture (VecType& p, VecType& q);

/**
 * @brief Kernel used into the projection process it takes 2 points and return a scalar
 * @param p : A point that will be compared with q
 * @param q : A point that will be compared with p
 */
template<typename VecType>
float rational_kernel (VecType& p, VecType& q);

/**
 * @brief This function returns the middle of two points.
 */
template <typename VecType>
VecType midpoint(VecType& a, VecType& b);

/**
 * @brief This function calculates the signed distance between the point q and the protection sphere of the node.
 */
template <typename statistics, typename VecType>
float signedDistanceToSphere (Octree <statistics, VecType> *node, VecType& q);

/**
 *
 * @brief This function takes a node of an octree and a point and returns a boolean
 * to say if this point is in the protection sphere
 */
template<typename statistics, typename VecType>
bool is_InProtectionSphere (Octree<statistics, VecType> *node, VecType& q);

/**
 * @brief This function computes the summation of two given statistics.
 */
template <typename statistics>
statistics sum_statistics (const statistics& a, const statistics& b);

/**
 * @brief This function returns the statistics given in input, multiplied by the factor w.
 */
template<typename statistics>
statistics weighted_statistics (statistics stats, float w);

/**
 *
 * @brief This function takes the father's node and one of its children and returns
 * the gamma function (Equation (7) in the MLoD's paper).
 * It calculates the distance between q and the protection sphere of each node (father and child).
 *
 * @param node : This is the current node that we treat.
 * @param child : This is the child node that we treat too.
 * @param q : This is the point that we want to project.
 *
 */
template<typename statistics, typename VecType>
float gamma_maj (Octree<statistics, VecType> *node, Octree<statistics, VecType> *child, VecType q);

/**
 * @brief Sub-methode of the projection that will recursivly go throught the octree to blend the stats of the algebraic sphere
 * @param octree : Root octree of the input point cloud
 * @param q : Point q that we want to project into the octree
 */
template<typename statistics, typename VecType>
statistics cumul_stats(Octree<statistics, VecType>* child, float (*kernel)(VecType& ,VecType& ), VecType& q);

/**
 * @brief Implementation of the projection methode from the MLoD paper.
 * @param octree : Root octree of the input point cloud
 * @param q : Point q that we want to project into the octree
 */
template<typename statistics, typename VecType, typename point>
std::pair<VecType, float> projection (Octree<statistics, VecType>* octree, float (*kernel)(VecType& ,VecType& ) ,VecType& q);
// point projection (Octree<statistics, VecType>* octree, float (*kernel)(VecType& ,VecType& ) ,VecType& q);
