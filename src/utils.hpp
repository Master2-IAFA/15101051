#pragma once

#include "Octree.hpp"
#include "PointSet.hpp"


template class Octree<statistics>;
typedef Octree<statistics> InputOctree;

InputOctree* generateOctree(int max_depth, PointSet *pc);


typedef Octree<statistics> InputOctree;


void fitOctree( int max_depth, InputOctree *octree, PointSet *pc );

std::vector<glm::vec3> build_cube_from_minmax(glm::vec3 min, glm::vec3 max);
