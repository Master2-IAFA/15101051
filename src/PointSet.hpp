#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <glm/glm.hpp>

using std::vector;
using std::string;

/** represents a point cloud 
 * @tparam point traits, typically positions and normals (glm::vec3 or vec2)
*/
template<typename point>
class PointSet {
public:
    PointSet () {}
    PointSet (vector<point> _points) { m_points = _points; }

    /***getter***/
    inline vector<point> getPoints () const& { return m_points; }

    /** read point cloud using OpenMesh primitive
     *
     * all files, no matter the format, are considered as point cloud. If file contains mesh, only vertices are loaded.
     * supports .ply .obj formats
     * @note if no normals are found, all normals are set to (1, 0, 0). erases former data and replaces with new read one
     * @warning dependency to OpenMesh library
     * @author linda
     * @param [in] filename file path (absolute or relative from src folder)
     */
    void readOpenMesh (string filename);

    /** compute the bounding box of current point cloud
     *
     * bounding box: cuboid that contains all points of the point cloud
     * @return min/max points (down left front/up right back corner points) of the box
     */
    std::pair<point, point> getBoundingBox ();
    void addPoint(point p) { m_points.push_back(p); }

private:
    vector<point> m_points;
};
