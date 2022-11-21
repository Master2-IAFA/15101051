#pragma once

#include <iostream>
#include <glm/glm.hpp>
#include "utils.t.hpp"

#define LAMBDA 1.3f

/**
 * @brief This function returns the middle of two points.
 */
template <typename VecType>
VecType midpoint(VecType& a, VecType& b);


template <typename statistics, typename VecType>
float signedDistanceToSphere (Octree <statistics, VecType> *node, VecType& q);

/**qualifier
 * @brief This function takes a node of an octree and a point and returns a boolean 
 * to say if this point is in the protection sphere
 */
template<typename statistics, typename VecType>
bool is_InProtectionSphere (Octree<statistics, VecType> *node, VecType& q);

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
point projection (Octree<statistics, VecType>* octree, float (*kernel)(VecType& ,VecType& ) ,VecType& q);
