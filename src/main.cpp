#include <iostream>

#include "polyscope/messages.h"
#include "polyscope/point_cloud.h"
#include "polyscope/pick.h"
#include "polyscope/polyscope.h"

#include "glm/gtx/string_cast.hpp"
#include "utils.hpp"

#include "debug.hpp"


int main(int argc, char **argv){
    //test_basic_polyscope();
    test_debug_readPly();

    return 0;
}



  bb_example = ps->getBoundingBox();
  std::cout << "bb_example : min = " << bb_example[0].pos.x << "," << bb_example[0].pos.y << "," << bb_example[0].pos.z ;
  std::cout << "bb_example : max = " << bb_example[1].pos.x << "," << bb_example[1].pos.y << "," << bb_example[1].pos.z ;

  std::vector<glm::vec3> points_bb;
  for (int i = 0 ; i < bb_example.size() ; ++i)
  {
    points_bb.push_back(bb_example[i].pos) ;
  }
  polyscope::registerPointCloud("bounding box", points_bb);
  auto f = polyscope::getPointCloud("bounding box");
  f->setEnabled(true);
}

void test_debug_subdivide () {
<<<<<<< Updated upstream
	Octree<int> tree (0, glm::vec3(0, 0, 0), glm::vec3(100, 100, 100));

	tree.subDivide();

	auto children = tree.getChildren();
	children[0]->subDivide();
	//TODO display tree
=======
	Octree<Data> tree (0, glm::vec3(0, 0, 0), glm::vec3(100, 100, 100));

	tree.subdivide();

	tree.getChildren[0]->subdivide();
	tree.getChildren[1]->subdivide();
	tree.getChildren[3]->subdivide();

	tree.getChildren[0]->getChildren[2]->subdivide();
	tree.getChildren[0]->getChildren[4]->subdivide();
}*/


int main(int argc, char **argv){
  std::cout << "Demarrage " << std::endl;
  std::cout <<  "\n Size " << bunny->getPoints().size() << std::endl;

  polyscope::init();
  polyscope::show();

  vector<point> my_points ;
  std::vector<glm::vec3> colors;

  // generate points
  for(float i = 0; i < 50; i++){
      for( float j = 0; j < 50; j++){
          float x = (i - 25) / 25.0f;
          float y = (j - 25) / 25.0f;
          points.push_back( glm::vec3( i/50.0f, exp( -(x*x) - (y*y)) , j/50.0f));
          colors.push_back( glm::vec3(0.0f, 0.0f, 0.0f) );
          my_points.pos.push_back(glm::vec3( i/50.0f, exp( -(x*x) - (y*y)) , j/50.0f));
          my_points.norm.push_back(glm::vec3(1.0, 1.0, 1.0));
      }
  }
  return 0;
}
