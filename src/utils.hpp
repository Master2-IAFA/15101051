#pragma once

#include "Octree.t.hpp"
#include "PointSet.hpp"
#include "glm/gtx/norm.hpp"

typedef struct {
  glm::vec3 position;
  glm::vec3 normal;
  double norm;
  double area;
  double pdn;
} statistics3d;

typedef Octree<statistics3d, glm::vec3> InputOctree;

InputOctree* generateInputOctree( int max_depth, PointSet<point3d> *pc );

void fitInputOctree( int max_depth, InputOctree *octree, std::vector<point3d> *points );

void statisticsAdd( statistics3d *stat, point3d point );
