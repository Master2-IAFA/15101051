#pragma once

#include "Octree.hpp"
#include "PointSet.hpp"

typedef Octree<statistics> InputOctree;

InputOctree* generateOctree (int max_depth, PointSet pc);
