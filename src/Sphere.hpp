#pragma once
#include "debug.hpp"

class Sphere{
  public :

  Sphere();

  /**
   * @brief function to compute center from algebraic sphere parameters
   */
  glm::vec3 get_center();

  /**
   * @brief function to compute radius from algebraic sphere parameters
   */
  float get_radius();

  /**
   * @brief function to fit an algebraic sphere to projected point according to the node's statistics
   */
  void fit_sphere_on_node(InputOctree * octree, PointSet * ps, glm::vec3 q);

  private:
    glm::vec3 center ;
    float radius ;

    glm::vec3 m_ul ;
    float m_uc ;
    float m_uq ;
  };
