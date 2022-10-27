#pragma once

#include <iostream>
#include "PointSet.hpp"
#include "utils.hpp"
#include "Octree.t.hpp"
#include <glm/glm.hpp>

#define LAMBDA 1.3f

/**
 * @brief This function returns the middle of two points.
 */
glm::vec3 midpoint(const glm::vec3& a, const glm::vec3& b);

/**qualifier
 * @brief This function takes a node of an octree and a point and returns a boolean 
 * to say if this point is in the protection sphere
 */
bool is_InProtectionSphere (InputOctree *node, glm::vec3& q);

/**
 * @brief Kernel used into the projection process it takes 2 points and return a scalar
 * @param p : A point that will be compared with q
 * @param q : A point that will be compared with p
 */
float gaussian_mixture (glm::vec3& p, glm::vec3& q);

/**
 * @brief Kernel used into the projection process it takes 2 points and return a scalar
 * @param p : A point that will be compared with q
 * @param q : A point that will be compared with p
 */
float rational_kernel (glm::vec3& p, glm::vec3& q);

/**
 * @brief Sub-methode of the projection that will recursivly go throught the octree to blend the stats of the algebraic sphere
 * @param octree : Root octree of the input point cloud
 * @param q : Point q that we want to project into the octree
 */
statistics cumul_stats(InputOctree* child, float (*kernel)(glm::vec3& ,glm::vec3& ), glm::vec3& q);

/**
 * @brief Implementation of the projection methode from the MLoD paper.
 * @param octree : Root octree of the input point cloud
 * @param q : Point q that we want to project into the octree
 */
point projection (InputOctree* octree, float (*kernel)(glm::vec3& ,glm::vec3& ) ,glm::vec3& q);
