#include "PointSet.hpp"

#include <stdlib.h> 

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
    //iterate on points to find max/min value on each coordinate
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
    
    //find max value between coordinates
    float x = abs(max_x - min_x);
    float y = abs(max_y - min_y);
    float z = abs(max_z - min_z);
    
    float max = (x > y && x > z)? x : ((y > z)? y : z);
    
    bb[0].pos.x = min_x ;
    bb[0].pos.y = min_y ;
    bb[0].pos.z = min_z ;

<<<<<<< Updated upstream
    bb[1].pos.x = (max_x - min_x)? max : (-1) * max;
    bb[1].pos.y = (max_y - min_y)? max : (-1) * max;
    bb[1].pos.z = (max_z - min_z)? max : (-1) * max;
    
=======
    bb[1].pos.x = max_x ;
    bb[1].pos.y = max_y ;
    bb[1].pos.z = max_z ;

    //a diagonal is generated, but it might not be a cube. In this cas, we have to augment it
    bb = augment(bb);
>>>>>>> Stashed changes
    return bb ;
}


/**
* reads a point set from a ply file
*
* erases former data if any and replaces with read point set
*/
void PointSet::readPly (string filename) {
    //file opening
    std::ifstream file (filename);
    if (!file) {
        throw std::runtime_error("cannot open ifstream " + filename);
        return;
    }

    string line;

    //header line
    if ( !std::getline( file, line ) ) {
        throw std::runtime_error("line reading error");
        return;
    }
    std::cout << line;

    if ( line.compare( "ply" ) != 0) {
        std::cout << "this file is not a ply file";
        return;
    }

    //format line
    std::getline( file, line );
    std::cout << line;

    //skip comments
    while ( std::getline( file, line ) ) {
        std::cout << line;
        if ( line.find( "comment" ) == string::npos ) {
            break;
        }
    }

    //get number of vertices
    int size = std::stoi(line.substr(15));
    m_points.clear();
    m_points.reserve(size);

    //get pos and norm positions in row while reading point records
    int acount = 0;
    int xpos = -1, ypos = -1, zpos = -1, nxpos = -1, nypos = -1, nzpos = -1;
    while ( std::getline( file, line ) ) {
        std::cout << line;
        if ( line.find( "property" ) == string::npos ) {
            break;
        }
        if ( line.find( "float x" ) != string::npos || line.find( "double x" ) != string::npos ) {
            xpos = acount;
        }

        if ( line.find( "float y" ) != string::npos || line.find( "double y" ) != string::npos ) {
            ypos = acount;
        }

        if ( line.find( "float z" ) != string::npos || line.find( "double z" ) != string::npos ) {
            zpos = acount;
        }

        if ( line.find( "float nx" ) != string::npos || line.find( "double nx" ) != string::npos ) {
            nxpos = acount;
        }

        if ( line.find( "float ny" ) != string::npos || line.find( "double ny" ) != string::npos ) {
            nypos = acount;
        }

        if ( line.find( "float nz" ) != string::npos || line.find( "double nz" ) != string::npos ) {
            nzpos = acount;
        }

        ++acount;
    }

    if ( (xpos == -1) || (ypos == -1) || (zpos == -1) ||
         (nxpos == -1) || (nypos == -1) || (nzpos == -1) ) {
        return;
    }

    //skip remaining header lines
    do {
        std::cout << line;
        if ( line.find( "end_header" ) != string::npos ) {
            break;
        }
    } while ( std::getline( file, line ) );

    std::cout << line;

    //read point records
    double num;
    vector<double> record;
    while ( file >> num ) {
        record.emplace_back( num );
        if ( record.size() == acount ) {
            point p;
            p.pos[0] = record[xpos];
            p.pos[1] = record[ypos];
            p.pos[2] = record[zpos];
            p.norm[0] = record[nxpos];
            p.norm[1] = record[nypos];
            p.norm[2] = record[nzpos];

            m_points.emplace_back(p);
            record.clear();
        }
    }
}
