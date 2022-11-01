#pragma once

#include <stdlib.h>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>

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
     * @note if no normals are found, all normals are set to (1, 0, 0). erases former data and replaces with new read one
     * @warning dependency to OpenMesh library
     * @param filename file path (absolute or relative from src folder)
     */
    void readOpenMesh (string filename);

    /** compute the bounding box of current point cloud
     *
     * bounding box: cubo�d that contains all points of the point cloud
     * @return min/max points (down left front/up right back corner points) of the box
     */
    vector<point> getBoundingBox ();
    void addPoint(point point) { m_points.push_back(point); }

    /**
     * @brief function to debug (prints pointset coords and normals)
     */
    void print_pointset_infos(OpenMesh::PolyMesh_ArrayKernelT<> mesh);


    //----------------------------fonctions de calcul de normales en fonction
    //---------------------------des faces et vertices d'un fichier (inutile pour
    //-------------------------le moment, à supprimer ou ignorer)

    /**
     * @brief function to compute face normals
     */
    glm::vec3 computeFaceNormal(std::array<double, 3> p1, std::array<double, 3> p2, std::array<double, 3> p3);

    /**
     * @brief function to compute vertices normals
     */
    std::vector<glm::vec3> calculateNormals(std::pair<std::vector<std::array<double, 3> >, std::vector<std::array<size_t, 3>>> unpackOpenMesh);

    /**
     * @brief function get vertices and normals list of mesh
     */
    std::pair<std::vector<std::array<double, 3> >, std::vector<std::array<size_t, 3>>>
    unpack_mesh(OpenMesh::PolyMesh_ArrayKernelT<> &mesh) ;

private:
  vector<point> m_points;
};
