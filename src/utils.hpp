#pragma once

#include "Octree.t.hpp"
#include "PointSet.t.hpp"
#include "glm/gtx/norm.hpp"

typedef struct {
  glm::vec3 position;
  glm::vec3 normal;
  double norm;
  double area;
  double pdn;
} statistics3d;

typedef struct {
  glm::vec2 position;
  glm::vec2 normal;
  double norm;
  double area;
  double pdn;
} statistics2d;

/** one point with positions and normals features
 */
typedef struct  {
    glm::vec3 pos;
    glm::vec3 norm;
} point3d;

typedef struct  {
    glm::vec2 pos;
    glm::vec2 norm;
} point2d;

template<typename statistics, typename point, typename VecType>
Octree<statistics, VecType>* generateInputOctree( int max_depth, PointSet<point> *pc );

template<typename statistics, typename point, typename VecType>
void fitInputOctree( int max_depth, Octree<statistics, VecType> *octree, std::vector<point> *points );

template<typename statistics, typename point>
void statisticsAdd( statistics *stat, point p );
