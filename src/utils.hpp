#include "Octree.hpp"

typedef struct  {
  glm::vec3 pos;
  glm::vec3 norm;
} point;

typedef struct  {
  glm::vec3 position;
  glm::vec3 normal;
  double norm;
  double area;
  double pdn;
} statistics;

typedef InputOctree Octree<statistics>;

InputOctree* generateOctree (int max_depth, PointSet pc);
