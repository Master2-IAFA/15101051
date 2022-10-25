#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <polyscope/polyscope.h>
#include <glm/glm.hpp>

using std::vector;
using std::string;

/** one point with positions and normals features
 */
typedef struct  {
  glm::vec3 pos;
  glm::vec3 norm;
} point;

/** represents a point cloud
 */
class PointSet {
public:
    PointSet () {}
    PointSet (vector<point> _points) { m_points = _points; }

    /***getter***/
    inline vector<point> getPoints () const& { return m_points; }

    /** read point cloud from .ply file
     * @note erases former point cloud data from variable and replaces it with read point cloud
     * @param filename file path (absolute or relative from src folder)
     */
    void readPly (string filename);

    /** read point cloud using OpenMesh primitive
     *
     * all files, no matter the format, are considered as point cloud. If file contains mesh, only vertices are loaded.
     * supports .ply .obj formats
     * @note if no normals are found, all normals are set to (1, 0, 0)
     * @warning dependency to OpenMesh library
     * @param filename file path (absolute or relative from src folder)
     */
    void readOpenMesh (string filename);

    /** compute the bounding box of current point cloud
     *
     * bounding box: cuboïd that contains all points of the point cloud
     * @return min/max points (down left front/up right back corner points) of the box
     */
    vector<point> getBoundingBox ();
    void addPoint(point point) { m_points.push_back(point); }

private:
  vector<point> m_points;
};
