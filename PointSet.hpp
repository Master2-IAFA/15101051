#include <iostream>
#include <fstream>
#include <vector>

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
  inline const *vector<point> getPoints { return &m_points; }
  void readPly (string filename);
  vector<point> getBoundingBox ();

private:
  std::vector<point> m_points;
}
