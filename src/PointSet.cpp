#include "PointSet.hpp"

float min(float x, float y)
{
  if (x < y) return x ;
  else return y ;
}

float max(float x, float y)
{
  if (x > y) return x ;
  else return y ;
}

void PointSet::print_pointset_infos(OpenMesh::PolyMesh_ArrayKernelT<> mesh)
{
  // move all vertices one unit length along it's normal direction
  for (OpenMesh::PolyMesh_ArrayKernelT<>::VertexIter v_it = mesh.vertices_begin();
       v_it != mesh.vertices_end(); ++v_it)
  {
    std::cout << "Vertex #"  << ": " << mesh.point( *v_it );
    std::cout << " Normal # " << mesh.point( *v_it ) << std::endl;
  }
}


//function that returns bounding box of point set
std::vector<point> PointSet::getBoundingBox() {
    std::vector<point> bb(2);
    //goal is to get min and max of every x y z axis
    float min_x ;
    float min_y ;
    float min_z ;

    float max_x ;
    float max_y ;
    float max_z ;

    int i ;

    //initialize min/max to first point coordinates if such point exists
    if (this->m_points.size() > 0) {
        min_x = m_points[0].pos[0];
        min_y = m_points[0].pos[1];
        min_z = m_points[0].pos[2];

        max_x = m_points[0].pos[0];
        max_y = m_points[0].pos[1];
        max_z = m_points[0].pos[2];
    }

    //iterate on points to find max/min value on each coordinate
    for (i = 0 ; i < this->m_points.size() ; ++i)
    {
      point p = this->m_points[i] ;

      min_x = min(min_x, p.pos.x) ;
      min_y = min(min_y, p.pos.y) ;
      min_z = min(min_z, p.pos.z) ;

      max_x = max(max_x, p.pos.x) ;
      max_y = max(max_y, p.pos.y) ;
      max_z = max(max_z, p.pos.z) ;

    }

    //find max value between coordinates
    float x = abs(max_x - min_x);
    float y = abs(max_y - min_y);
    float z = abs(max_z - min_z);

    float max = (x > y && x > z)? x : ((y > z)? y : z);

    bb[0].pos.x = min_x ;
    bb[0].pos.y = min_y ;
    bb[0].pos.z = min_z ;

    bb[1].pos.x = min_x + max;
    bb[1].pos.y = min_y + max;
    bb[1].pos.z = min_z + max;

    return bb ;
}


void PointSet::readOpenMesh (string filename) {
    m_points.clear();

    OpenMesh::PolyMesh_ArrayKernelT<> mesh;

    if (!OpenMesh::IO::read_mesh(mesh, filename)) {
        return;
    }

    //auto unpackOpenMesh = unpack_mesh(mesh);
    //std::vector<glm::vec3> normals = calculateNormals(unpackOpenMesh);

    for (auto v = mesh.vertices_sbegin(); v != mesh.vertices_end(); ++v) {
        auto current_point = mesh.point(*v);
        point p;
        p.pos[0] = current_point[0];
        p.pos[1] = current_point[1];
        p.pos[2] = current_point[2];

        if (mesh.has_vertex_normals()) {
            auto n = mesh.normal(*v);
            p.norm[0] = n[0];
            p.norm[1] = n[1];
            p.norm[2] = n[2];
        }
        else {
          p.norm = glm::vec3(0.0, 0.0, 1.0) ;
        }
        this->m_points.emplace_back(p);
    }


}

void PointSet::readPly (string filename) {
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


//-----------------------------------------------------------------------------

//fonctions de Lou pour calculer les normales d'un fichier ply en fonction de ses
//faces et de ses vertices (pas très utile pour le moment malheureusement)

glm::vec3 PointSet::computeFaceNormal(std::array<double, 3> p1, std::array<double, 3> p2, std::array<double, 3> p3) {
    // Uses p2 as a new origin for p1,p3
    glm::vec3 vec_p3 = { p3[0], p3[1], p3[2] };
    glm::vec3 vec_p2 = { p2[0], p2[1], p2[2] };
    glm::vec3 vec_p1 = { p1[0], p1[1], p1[2] };

    auto a = vec_p3 - vec_p2;
    auto b = vec_p1 - vec_p2;
    // Compute the cross product a X b to get the face normal
    return glm::normalize(glm::cross(a, b));
}

std::vector<glm::vec3> PointSet::calculateNormals(std::pair<std::vector<std::array<double, 3> >, std::vector<std::array<size_t, 3>>> unpackOpenMesh)
{
    auto vertices = unpackOpenMesh.first;
    auto faces = unpackOpenMesh.second;
    std::vector<glm::vec3> normals ;
    // For each face calculate normals and append it
    // to the corresponding vertices of the face
    for (unsigned int i = 0; i < faces.size(); i ++)
    {
        std::array<double, 3> A = vertices[faces[i][0]];
        std::array<double, 3> B = vertices[faces[i][1]];
        std::array<double, 3> C = vertices[faces[i][2]];
        glm::vec3 normal = computeFaceNormal(A, B, C);
        normals[faces[i][0]] += normal;
        normals[faces[i][1]] += normal;
        normals[faces[i][2]] += normal;
    }
    // Normalize each normal
    for (unsigned int i = 0; i < normals.size(); i++)
        normals.push_back(glm::normalize(normals[i]));

    std::cout << "normal size in function = " << faces.size() << "\n" ;
    return normals ;
}

std::pair<std::vector<std::array<double, 3> >, std::vector<std::array<size_t, 3>>>
PointSet::unpack_mesh(OpenMesh::PolyMesh_ArrayKernelT<> &mesh)
{
  std::vector<std::array<double, 3> > vertices ;
  std::vector<std::array<size_t, 3>> faces ;

  for( auto p : mesh.vertices()){
      auto point = mesh.point( p );
      std::array<double, 3> arrayPoint;
      arrayPoint[0] = point[0];
      arrayPoint[1] = point[1];
      arrayPoint[2] = point[2];
      vertices.push_back( arrayPoint );
    }

    int temp = 0;
    for (auto f: mesh.faces()) {
          std::array<size_t, 3> arrayFace;
          for (auto fv_it = mesh.fv_iter(f); fv_it.is_valid(); ++fv_it) {
              arrayFace[temp] = fv_it->idx();
              temp++;
          }
          faces.push_back( arrayFace );
          temp = 0;
      }

      //debug
      /*for (int i = 0 ; i < vertices.size() ; ++i)
      {
        std::cout << vertices[i][0] << vertices[i][1] << vertices[i][2] ;
      }

      for (int i = 0 ; i < faces.size() ; ++i)
      {
        std::cout << faces[i][0] << faces[i][1] << faces[i][2] << "\n" ;
      }*/

      return std::make_pair(vertices, faces);
}
