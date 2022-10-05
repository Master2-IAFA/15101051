/*

#include "PointSet.hpp"

//function that returns bounding box of point set
vector<point> getBoundingBox () {

}

/**
* reads a point set from a ply file
*//*
void PointSet::readPly (string filename) {
  //file opening
  ifstream file (filename);
  if (!file) {
    #erreur
  }

  string line;

  //header line
  if ( !std::getline( stream, line ) ) {
    #erreur
    return -1;
  }
  if ( std::compare( line, "ply" ) != 0) {
    std::cout << "this file is not a ply file";
    return;
  }

  //format line
  std::getline( stream, line );
  while ( line.find( "comment" ) != std::fpos );
}

*/
