#include "debug.hpp"

void pointSetToPolyscope( std::string name, PointSet *ps ){

    std::vector<point> points = ps->getPoints();

    std::vector<glm::vec3> position(points.size());
    std::vector<glm::vec3> normal(points.size());

    #pragma omp parallel for
    for(int i = 0; i < points.size(); i++){
        position[i] = points[i].pos;
        normal[i] = points[i].norm;
    }

    polyscope::PointCloud *pointCloud = polyscope::registerPointCloud(name, position);
    pointCloud->addVectorQuantity("normal", normal);
}

//function that takes minmax of a cube a returns 8 coordinates of cube
std::vector<glm::vec3> build_cube_from_minmax(glm::vec3 min, glm::vec3 max)
{
  //float len_body_diag = (max-min).length() ;
  //float a = len_body_diag/sqrt(3) ;

  glm::vec3 tfl = glm::vec3(min.x, min.y + max.y, min.z ) ;
  glm::vec3 tfr = glm::vec3(min.x + max.x, min.y + max.y, min.z) ;
  glm::vec3 tbl = glm::vec3(min.x, min.y + max.y, min.z + max.z ) ;
  glm::vec3 tbr = glm::vec3(min.x + max.x, min.y + max.y, min.z + max.z) ;
  glm::vec3 bfl = glm::vec3(min.x, min.y, min.z) ;
  glm::vec3 bfr =glm::vec3(min.x + max.x, min.y, min.z) ;
  glm::vec3 bbl  = glm::vec3(min.x, min.y, min.z + max.z) ;
  glm::vec3 bbr = glm::vec3(min.x + max.x, min.y, min.z + max.z) ;

  std::vector<glm::vec3> cube ;
  cube.push_back(tfl);
  cube.push_back(tfr);
  cube.push_back(tbl);
  cube.push_back(tbr);
  cube.push_back(bfl);
  cube.push_back(bfr);
  cube.push_back(bbl);
  cube.push_back(bbr);
  return cube ;

}