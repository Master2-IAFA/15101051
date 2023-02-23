#pragma once
#include "PointSet.hpp"

#include <stdlib.h>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>

inline float min(float x, float y) { return (x < y)? x : y; }

inline float max(float x, float y) { return (x > y)? x : y; }

/** function that returns bounding box of point set */
template<typename point>
std::pair<point, point> PointSet<point>::getBoundingBox() {
    //goal is to get min and max of every x y z axis
    std::vector<float> min_vec;
    std::vector<float> max_vec;

    //initialize min/max to first point coordinates if such point exists
    if ( this->m_points.size() > 0 ) {
        for ( size_t i = 0 ; i < m_points[0].pos.length() ; ++i ) {
            min_vec.emplace_back( m_points[0].pos[i] );
            max_vec.emplace_back( m_points[0].pos[i] );
        }
    }

    //iterate on points to find max/min value on each coordinate
    for ( size_t i = 1 ; i < this->m_points.size() ; ++i ) {
        point p = this->m_points[i] ;

        for ( size_t i = 0 ; i < min_vec.size() ; ++i ) {
            min_vec[i] = min( min_vec[i], p.pos[i] );
            max_vec[i] = max( max_vec[i], p.pos[i] );
        }
    }

    //find max value between coordinates
    std::vector<float> distances;
    for ( size_t i = 0 ; i < min_vec.size() ; ++i ) {
        distances.emplace_back( abs( max_vec[i] - min_vec[i] ) );
    }

    float max_val = distances[0];
    for ( size_t i = 1 ; i < distances.size() ; ++i ) {
        max_val = max( max_val, distances[i] );
    }

    point p1, p2;
    for ( size_t i = 0 ; i < min_vec.size() ; ++i ) {
        p1.pos[i] = min_vec[i];
        p2.pos[i] = min_vec[i] + max_val;
    }

    return std::make_pair(p1, p2);
}

template<typename point>
void PointSet<point>::readOpenMesh (string filename) {
    m_points.clear();

    OpenMesh::PolyMesh_ArrayKernelT<> mesh;

    // request vertex normals, so the mesh reader can use normal information
    // if available
    // mesh.release_vertex_colors();
    mesh.request_vertex_normals();
    // assure we have vertex normals
    if (!mesh.has_vertex_normals()) {
        std::cerr << "ERROR: Standard vertex property 'Normals' not available!\n";
        exit(1);
    }
    OpenMesh::IO::Options opt (OpenMesh::IO::Options::VertexNormal);
    if (!OpenMesh::IO::read_mesh(mesh, filename, opt)) {
        return;
    }

    for (auto v = mesh.vertices_sbegin(); v != mesh.vertices_end(); ++v) {
        point p;

        auto current_point = mesh.point(*v);
        for (int i = 0;i < p.pos.length();++i) {
            p.pos[i] = current_point[i];
        }

        auto n = mesh.normal(*v);
        for (int i = 0;i < p.norm.length();++i) {
            p.norm[i] = n[i];
        }

        this->m_points.emplace_back(p);
    }

    mesh.release_vertex_normals();
}