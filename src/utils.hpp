#include "Octree.hpp"

typedef struct  {
  glm::vec3 pos;
  glm::vec3 norm;
} point;

typedef struct  {

} statistics;

typedef InputOctree Octree<statistics>;

InputOctree* generateOctree (int max_depth, PointSet pc);
