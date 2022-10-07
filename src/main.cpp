
#include "polyscope/messages.h"
#include "polyscope/point_cloud.h"
#include "polyscope/pick.h"
#include "polyscope/polyscope.h"

#include "glm/gtx/string_cast.hpp"
#include "utils.hpp"


//function to vizualise bounding box of point cloud (Lou)
void test_debug_bounding_box(std::vector<glm::vec3> points);

float kernelSize = 1.0f;
polyscope::PointCloud* psCloud;
std::vector<glm::vec3> points;
std::vector<glm::vec3> colors;



int main(int argc, char **argv){

    //TEST IS POINT IN
    //Octree * octree = new Octree()

    polyscope::init();

    //std::vector<glm::vec3> points;

    //polyscope::registerPointCloud("octree", cube);
    Octree<glm::vec3> * octree ;
    octree = new Octree<glm::vec3>(10, glm::vec3(0.0, 0.0, 0.0), glm::vec3(3.0, 3.0, 3.0));
    bool is_inside = octree->isPointIn(glm::vec3(1.0, 1.0, 1.0));

    // generate points
    for(float i = 0; i < 50; i++){
        for( float j = 0; j < 50; j++){
            float x = (i - 25) / 25.0f;
            float y = (j - 25) / 25.0f;
            points.push_back( glm::vec3( i/50.0f, exp( -(x*x) - (y*y)) , j/50.0f));
            colors.push_back( glm::vec3(0.0f, 0.0f, 0.0f) );
        }
    }
    //test_debug_bounding_box(points);
    //test_debug_subdivide();

    polyscope::registerPointCloud("gauss", points);
    polyscope::show();

    return 0;
}

//-----------------------------VIZUALISE BB OF POINT CLOUD----------------------
void test_debug_bounding_box(std::vector<glm::vec3> points)
{
  //------------------test getBoundingBox (Lou)
  vector<point> my_points ;
  std::vector<glm::vec3> colors;
  //glm::vec3 useless_norm = (0.0, 0.0, 0.0);
  for (int i = 0 ; i < points.size() ; ++i)
  {
    point p;
    p.pos = points[i] ;
    p.norm = glm::vec3(0.0, 0.0, 0.0) ;
    my_points.push_back(p);
  }

  PointSet * ps = new PointSet(my_points) ;
  vector<point> bb_example ;

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
	Octree<int> tree (0, glm::vec3(0, 0, 0), glm::vec3(100, 100, 100));

	tree.subDivide();

	auto children = tree.getChildren();
	children[0]->subDivide();
	//TODO display tree
}
