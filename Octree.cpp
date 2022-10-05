#include "Octree.hpp"

template<typename Data>
Octree<Data>::Octree(int depth, double half_dimension, glm::vec3 center): _depth(depth), _half_dimension(half_dimension), _center(center){

}

template<typename Data>
Octree<Data>::~Octree(){
  _father = nullptr;
  delete _data;
  for(int i = 0; i < 8; i++){
    delete _children[i];
  }
}

template<typename Data>
void Octree<Data>::subDivise(){
  double children_half_dimension = _half_dimension / 2;
  for(int i = 0; i < 8; i++){
    int right = (0b001 & i) && 0? -1 : +1; //left c'est 0
    int back  = (0b010 & i) && 0? -1 : +1; //front c'est 0
    int top   = (0b100 & i) && 0? -1 : +1; //top c'est 0
    glm::vec3 vec(right, top, back);
    Octree children = new Octree(_depth + 1, children_half_dimension, vec * _center);
    children->setFather(this);
    _children[i] = children;
  }
}
