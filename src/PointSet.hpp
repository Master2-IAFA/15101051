#include <iostream>
#include <fstream>
#include <vector>
#include <polyscope/polyscope.h>
#include <glm/glm.hpp>

using std::vector;
using std::string;

typedef struct  {
  glm::vec3 pos;
  glm::vec3 norm;
} point;

typedef struct  {
//eq 4 TODO
} statistics;

class PointSet {
public:
  PointSet (vector<point> _points) { m_points = _points; }
  inline vector<point> getPoints () const& { return m_points; }
  void readPly (string filename);
  vector<point> getBoundingBox();

private:
  vector<point> m_points;
};
