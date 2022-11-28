#pragma once

#include "Octree/BaseOctree.t.hpp"
#include "Octree/InputOctree.t.hpp"
#include "Octree/Octree.t.hpp"

#include "AlgebraicSphere.t.hpp"
#include "PointSet.t.hpp"

#include "utils.t.hpp"

typedef InputOctree< glm::vec3, statistics3d, point3d > InputOctree3D;
typedef InputOctree< glm::vec2, statistics2d, point2d > InputOctree2D;

typedef Octree< statistics3d, glm::vec3 > Octree3D;
typedef Octree< statistics2d, glm::vec2 > Octree2D;

typedef AlgebraicSphere< glm::vec3, statistics3d > AlgebraicSphere3D;
typedef AlgebraicSphere< glm::vec2, statistics2d > AlgebraicSphere2D;

typedef PointSet< point3d > PointSet3D;
typedef PointSet< point2d > PointSet2D;