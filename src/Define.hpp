#pragma once

#include "Octree/BaseOctree.t.hpp"
#include "Octree/InputOctree.t.hpp"
#include "Octree/Octree.t.hpp"

#include "AlgebraicSphere.t.hpp"
#include "PointSet.t.hpp"

#include "utils.t.hpp"

using InputOctree3D = InputOctree< glm::vec3, statistics3d, point3d >;
using InputOctree2D = InputOctree< glm::vec2, statistics2d, point2d >;

using Octree3D = Octree< statistics3d, glm::vec3 >;
using Octree2D = Octree< statistics2d, glm::vec2 >;

template< class T >
using BaseOctree3D = BaseOctree< statistics3d, glm::vec3, T >;

template< class T >
using BaseOctree2D = BaseOctree< statistics2d, glm::vec2, T >;

using AlgebraicSphere3D = AlgebraicSphere< glm::vec3, statistics3d >;
using AlgebraicSphere2D = AlgebraicSphere< glm::vec2, statistics2d >;

using PointSet3D = PointSet< point3d >;
using PointSet2D = PointSet< point2d >;