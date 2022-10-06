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

//template<class Data>
/*void Octree<Data>::subDivise(){
  double children_half_dimension = _half_dimension / 2;
  for(int i = 0; i < 8; i++){
    int right = (0b001 & i) != 0? +1 : -1; //left c'est 0
    int back  = (0b010 & i) != 0? +1 : -1; //front c'est 0
    int top   = (0b100 & i) != 0? -1 : +1; //top c'est 0
    glm::vec3 vec(right, top, back);
    vec = glm::normalize(vec);
    std::cout << right << " " << back << " " << top << std::endl;
    Octree<Data> *children = new Octree<Data>(_depth + 1, children_half_dimension,  _center + (vec * glm::vec3(children_half_dimension)));
    children->setFather(this);
    _children[i] = children;
  }
}*/

template<class Data>
std::vector<Octree<Data>*> Octree<Data>::getAtDepth(int depth){
  std::vector<Octree<Data>*> octree;
  std::vector<Octree<Data>*> stack;
  stack.push_back(this);
  while(!stack.empty()){  
    auto o = stack.back();
    stack.pop_back();
    if( o->getDepth() == depth ){
      octree.push_back(o);
    }else{
      auto children = o->getChildren();
      for(int i = 0; i < 8; i++){
        if(children[i] != nullptr){
          stack.push_back(children[i]);
        }
      }
    }
  }
  return octree;
}
