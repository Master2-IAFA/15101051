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
