#pragma once
#include "PointSet.hpp"

#include <stdlib.h>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>

inline float min(float x, float y) { return (x < y)? x : y; }

inline float max(float x, float y) { return (x > y)? x : y; }

/** function that returns bounding box of point set
 */
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
    mesh.request_vertex_normals();
   p.norm[i] = n[i];

    // assure we have vertex normals
    if (!mesh.has_vertex_normals())
    {
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

template<typename point>
void PointSet<point>::readPly (string filename) {
    m_points.clear();

    //file opening
    std::ifstream file (filename, std::ios::binary);
    if (!file) {
        throw std::runtime_error("cannot open ifstream " + filename);
        return;
    }

    string line;

    //header line
    if ( !std::getline( file, line ) ) {
        throw std::runtime_error("line reading error");
        return;
    }
    std::cout << line << std::endl;

    if ( line.compare( "ply" ) != 0) {
        std::cout << "this file is not a ply file";
        return;
    }

    //format line
    std::getline( file, line );
    std::cout << line << std::endl;

    //skip comments
    while ( std::getline( file, line ) ) {
        std::cout << line << std::endl;
        if ( line.find( "comment" ) == string::npos ) {
            break;
        }
    }

    //get number of vertices
    int size = std::stoi(line.substr(15));
    m_points.clear();
    m_points.reserve(size);

    //get pos and norm positions in row while reading point records
    int acount = 0;
    int xpos = -1, ypos = -1, zpos = -1, nxpos = -1, nypos = -1, nzpos = -1;
    while ( std::getline( file, line ) ) {
        std::cout << line << std::endl;
        if ( line.find( "property" ) == string::npos ) {
            break;
        }
        if ( line.find( "float x" ) != string::npos || line.find( "double x" ) != string::npos ) {
            xpos = acount;
        }

        if ( line.find( "float y" ) != string::npos || line.find( "double y" ) != string::npos ) {
            ypos = acount;
        }

        if ( line.find( "float z" ) != string::npos || line.find( "double z" ) != string::npos ) {
            zpos = acount;
        }

        if ( line.find( "float nx" ) != string::npos || line.find( "double nx" ) != string::npos ) {
            nxpos = acount;
        }

        if ( line.find( "float ny" ) != string::npos || line.find( "double ny" ) != string::npos ) {
            nypos = acount;
        }

        if ( line.find( "float nz" ) != string::npos || line.find( "double nz" ) != string::npos ) {
            nzpos = acount;
        }

        ++acount;
    }

    if ( (xpos == -1) || (ypos == -1) || (zpos == -1) ||
         (nxpos == -1) || (nypos == -1) || (nzpos == -1) ) {
        return;
    }

    //skip remaining header lines
    do {
        std::cout << line << std::endl;
        if ( line.find( "end_header" ) != string::npos ) {
            break;
        }
    } while ( std::getline( file, line ) );

    std::cout << line << std::endl;

    //read point records
    double num;
    vector<double> record;
    while ( file >> num ) {
        record.emplace_back( num );
        if ( record.size() == acount ) {
            point p;
            p.pos[0] = record[xpos];
            p.pos[1] = record[ypos];
            p.pos[2] = record[zpos];
            p.norm[0] = record[nxpos];
            p.norm[1] = record[nypos];
            p.norm[2] = record[nzpos];

            m_points.emplace_back(p);
            record.clear();
        }
    }
}
