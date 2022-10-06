#include <iostream>
#include <polyscope/polyscope.h>
                             //
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
  Octree(int depth, double half_dimension, glm::vec3 center);
  ~Octree();

  /**
  * @brief split the octree into 8 childrens, and link this as their father.
  */
  void subDivise();

  /***** setters ******/
  inline void setFather( Octree<Data> *father ){ _father = father; }
  inline void setData( Data data ){ _data = data; }

  /***** getters ******/
  inline const Data getData(){ return _data; }
  inline glm::vec3 getCenter(){ return _center; }
  inline double getHalfDimension(){ return _half_dimension; }
  inline int getDepth(){ return _depth; }


private:

  Octree<Data> *_father = nullptr;
  Octree<Data> *_children[8];
  int _depth;
  double _half_dimension;
  glm::vec3 _center;
  Data _data;

};
