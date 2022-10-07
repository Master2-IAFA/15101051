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
} statistics;

typedef Octree<statistics> InputOctree;

InputOctree* generateInputOctree( int max_depth, PointSet *pc );

void fitInputOctree( int max_depth, InputOctree *octree, std::vector<point> points );

void statisticsAdd( statistics *stat, point point );
