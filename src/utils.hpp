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


/**
 * @brief this function prints the informations contained in this stats.
 */
template <typename statistics> 
void display_statistics (statistics stats);

template<typename statistics, typename point, typename VecType>
Octree<statistics, VecType>* generateInputOctree( int max_depth, PointSet<point> *pc );

template<typename statistics, typename point, typename VecType>
void fitInputOctree( int max_depth, Octree<statistics, VecType> *octree, std::vector<point> *points );

/**
 * @brief This function updates the given stats with the information of the given point.
 *
 */
template<typename statistics, typename point>
void statisticsAdd( statistics *stat, point p );

/**
 * @brief This function takes algebraic parameters and returns the radius of the geometric sphere.
 */
template<typename VecType>
float get_radius(float m_uc, VecType m_ul, float m_uq, VecType center);

/**
 * @brief This function takes algebraic parameters and returns the center of the geometric sphere.
 */
template<typename VecType>
VecType get_center(float m_uc, VecType m_ul, float m_uq);

/**
 * @brief This function takes statistics of a node or aggregated statistics and returns the geometric parameters of the sphere.
 * It takes also a kernel.
 * @return : The pair représente in x.first the center of the geometric sphere, and in x.second the radius of it.
 */
template<typename statistics, typename VecType>
std::pair<VecType, float> fit_algebraic_sphere(statistics stat, VecType q, float (*kernel)(VecType&,VecType&));

/**
 * @brief This function takes the geometric parameters of the sphere and a point to project on it, and returns the new projected point.
 */
template<typename VecType>
VecType project_point (std::pair<VecType, float> sphereInfos, VecType q);