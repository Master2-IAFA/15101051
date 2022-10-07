#include "Octree.hpp"

template class Octree<int>;

template<class Data>
Octree<Data>::Octree(int depth, glm::vec3 min, glm::vec3 max){
    _min = min;
    _depth = depth;
    _max = max;
}

template<class Data>
Octree<Data>::~Octree(){
    _father = nullptr;
    //delete _data;
    for(int i = 0; i < 8; i++){
    delete _children[i];
    }
}

template<class Data>
void Octree<Data>::subDivide(){
    glm::vec3 center ((_min[0] + _max[0]) / 2,
    	(_min[1] + _max[1]) / 2,
    	(_min[0] + _max[0]) / 2);

    _children[0] = new Octree<Data>(_depth + 1, center, glm::vec3(_min[0], _min[1], _max[2]));
    _children[1] = new Octree<Data>(_depth + 1, center, glm::vec3(_max[0], _min[1], _max[2]));
    _children[2] = new Octree<Data>(_depth + 1, center, glm::vec3(_min[0], _max[1], _max[2]));
    _children[3] = new Octree<Data>(_depth + 1, center, _max);
    _children[4] = new Octree<Data>(_depth + 1, _min, center);
    _children[5] = new Octree<Data>(_depth + 1, glm::vec3(_max[0], _min[1], _min[2]), center);
    _children[6] = new Octree<Data>(_depth + 1, glm::vec3(_min[0], _max[1], _min[2]), center);
    _children[7] = new Octree<Data>(_depth + 1, glm::vec3(_max[0], _max[1], _min[2]), center);

    _children[0]->setFather(this);
    _children[1]->setFather(this);
    _children[2]->setFather(this);
    _children[3]->setFather(this);
    _children[4]->setFather(this);
    _children[5]->setFather(this);
    _children[6]->setFather(this);
    _children[7]->setFather(this);

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

template<class Data>
bool Octree<Data>::isPointIn(glm::vec3 p)
{
  glm::vec3 min = this->getMin();
  glm::vec3 max = this->getMax();
  //if you want to vizualise cube and point on polyscope
  //std::vector<glm::vec3> cube = build_cube_from_minmax(min, max);
  //cube.push_back(p) ;

  if ((p.x > min.x) && (p.x < max.x) &&
    (p.y > min.y) && (p.y < max.y) &&
    (p.z > min.z) && (p.z < max.z))
    {
      std::cout << "point is inside \n" ;
      return true ;
    }
    std::cout << "point is outside \n" ;
    return false ;
}

//TO DELETE (Lou en a besoin pour déclarer un octree dans le main)
template class Octree<glm::vec3>;
