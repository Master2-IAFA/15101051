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
    bb[0].pos.x = min_x ;
    bb[0].pos.y = min_y ;
    bb[0].pos.z = min_z ;

    bb[1].pos.x = max_x ;
    bb[1].pos.y = max_y ;
    bb[1].pos.z = max_z ;
    return bb ;

}


/**
* reads a point set from a ply file
*/
/*void PointSet::readPly (string filename) {
  //file opening
  ifstream file (filename);
  if (!file) {
    //erreur
  }

  string line;

  //header line
  if ( !std::getline( stream, line ) ) {
    //erreur
    return -1;
  }
  if ( std::compare( line, "ply" ) != 0) {
    std::cout << "this file is not a ply file";
    return;
  }

  //format line
  std::getline( stream, line );
  while ( line.find( "comment" ) != std::fpos );
}*/
