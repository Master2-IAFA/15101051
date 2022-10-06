#pragma once

#include "Octree.hpp"
#include "PointSet.hpp"
#include "glm/gtx/norm.hpp"


template class Octree<statistics>;
typedef Octree<statistics> InputOctree;

InputOctree* generateInputOctree(int max_depth, PointSet *pc);

void fitInputOctree( int max_depth, InputOctree *octree, std::vector<point> points );
void statisticsAdd(statistics *stat, point point);