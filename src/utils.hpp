#pragma once

#include "glm/gtx/norm.hpp"
#include "Octree.t.hpp"
#include "PointSet.t.hpp"
#include "blending.t.hpp"


typedef struct {
  // P_alpha
  glm::vec3 position = glm::vec3 (0.0f);
  // N_alpha
  glm::vec3 normal = glm::vec3(0.0f);
  // P_beta
  double norm = 0;
  //sigma
  double area = 0;
  // Pn_beta
  double pdn = 0;
} statistics3d;

typedef struct {
  glm::vec2 position = glm::vec2(0.0f);
  glm::vec2 normal = glm::vec2(0.0f);
  double norm = 0;
  double area = 0;
  double pdn = 0;
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

// template <typename statistics, typename VecType>
// void init_statistics (statistics *s);

template <typename statistics> 
void display_statistics (statistics stats);

template<typename statistics, typename point, typename VecType>
Octree<statistics, VecType>* generateInputOctree( int max_depth, PointSet<point> *pc );

template<typename statistics, typename point, typename VecType>
void fitInputOctree( int max_depth, Octree<statistics, VecType> *octree, std::vector<point> *points );

template<typename statistics, typename point>
void statisticsAdd( statistics *stat, point p );
