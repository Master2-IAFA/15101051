#pragma once

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

/**
 * @brief this function prints the informations contained in this stats.
 */
template <typename statistics> 
void display_statistics (statistics stats);

/**
 * @brief This function updates the given stats with the information of the given point.
 *
 */
template<typename statistics, typename point>
void statisticsAdd( statistics *stat, point p );

/**
 * @brief This function computes the summation of two given statistics.
 */
template <typename statistics>
statistics sum_statistics (const statistics& a, const statistics& b);

/**
 * @brief This function returns the statistics given in input, multiplied by the factor w.
 */
template<typename statistics>
statistics weighted_statistics (statistics stats, float w);