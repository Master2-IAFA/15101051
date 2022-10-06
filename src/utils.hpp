#pragma once

#include "Octree.hpp"
#include "PointSet.hpp"

template class Octree<statistics>;
typedef Octree<statistics> InputOctree;

InputOctree* generateOctree(int max_depth, PointSet *pc);

void fitOctree( int max_depth, InputOctree *octree, PointSet *pc );
