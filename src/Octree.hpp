#pragma once

#include <iostream>
#include <polyscope/polyscope.h>

#define TOP_FRONT_LEFT     0 // 0b000
#define TOP_FRONT_RIGHT    1 // 0b001
#define TOP_BACK_LEFT      2 // 0b010
#define TOP_BACK_RIGHT     3 // 0b011
#define BOTTOM_FRONT_LEFT  4 // 0b100
#define BOTTOM_FRONT_RIGHT 5 // 0b101
#define BOTTOM_BACK_LEFT   6 // 0b110
#define BOTTOM_BACK_RIGHT  7 // 0b111

/*
*
*
*/
template<typename Data>
class Octree{

public:
  Octree(int depth, glm::vec3 min, glm::vec3 max);
  ~Octree();

  /**
  * @brief split the octree into 8 childrens, and link this as their father.
  */
  void subDivide();

  /**
   * @brief get all the octrees at the given depth
   *
   */
  std::vector<Octree<Data>*> getAtDepth(int depth);
  bool isPointIn(glm::vec3 p);

  /***** setters ******/
  inline void setFather( Octree<Data> *father ){ _father = father; }
  inline void setData( Data data ){ _data = data; }

  /***** getters ******/
  inline const Data getData(){ return _data; }
  inline glm::vec3 getMin(){ return _min; }
  inline glm::vec3 getMax(){ return _max; }
  inline int getDepth(){ return _depth; }
  inline Octree<Data> **getChildren(){ return _children; }


private:

  std::vector<Octree<Data>*> pGetAtDepth(int depth, std::vector<Octree<Data>*> vector);

  Octree<Data> *_father{nullptr};
  Octree<Data> *_children[8];
  int _depth;
  glm::vec3 _min ;
  glm::vec3 _max ;
  Data _data;

};
