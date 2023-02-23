#pragma once

#include "AlgebraicSphere.t.hpp"
#include "Octree/InputOctree.t.hpp"

#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"

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

/** one point with positions and normals features */
typedef struct  {
    glm::vec3 pos;
    glm::vec3 norm;
} point3d;

typedef struct  {
    glm::vec2 pos;
    glm::vec2 norm;
} point2d;

/** PROJECTION OPERATOR */
template<typename statistics, typename point, typename VecType, typename PointType>
AlgebraicSphere<VecType, statistics> projection (InputOctree<VecType, statistics, PointType> *octree, float (*kernel)(VecType&,VecType&) ,VecType& q);

/** this function prints the informations contained in this stats.*/
template <typename statistics> 
void display_statistics (statistics stats);

/** This function updates the given stats with the information of the given point.*/
template<typename statistics, typename point>
void statisticsAdd( statistics *stat, point p );

/** This function computes the summation of two given statistics. */
template <typename statistics>
statistics sum_statistics (const statistics& a, const statistics& b);

/** This function returns the statistics given in input, multiplied by the factor w. */
template<typename statistics>
statistics weighted_statistics (statistics stats, float w);

/** generate a .ply file containing a sampeled 3D gaussian
 * @author linda
*/
void generate_gaussian ();

/** generate a .ply file containing a sampeled 2D gaussian
 * @author linda
 * @param [in] nbSamples number of points in resulting point cloud
 * @param [in] direction 0: left 1: down 2: right 3: top
*/
void generate2dGaussian (int nbSamples, int direction);

/** returns the number of different bits between two integers
 *
 * the number of bits is deduced from the greatest number. For
 * example, if 2 and 5 are compared, we consider 3 bits (as 5
 * needs 3 bits for binary encoding)
 * 
 * @author linda
*/
int bitDiff (unsigned int n, unsigned int m);

/** create a hypercube (square/cube) from its diagonal positions
 * @param min first diagonal coordinate
 * @param max second diagonal coordinate
 * @return hypercube coordinates (4/8 coordinates)
 */
template< class VecType >
std::vector<VecType> build_cube_from_minmax(VecType min, VecType max);

/** given min and max of cube, draws cube on polyscope
 * @param [in] name : name of polyscope window
 * @param [in] min : min point of cube
 * @param [in] max : max point of cube
 */
template<class VecType>
void drawCube(std::string name, VecType min, VecType max);

/** converts the point cloud \p ps to a polyscope drawable one
 * @param [in] name point cloud name in the GUI
 * @param [in] ps point cloud to render
 */
template<class VecType, class PointType>
polyscope::PointCloud* pointSetToPolyscope(std::string name, PointSet<PointType> *ps);
