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
    //test_debug_readPly();
    //drawCube(glm::vec3(0.0, 0.0, 0.0), glm::vec3(3.0, 3.0, 3.0));

    polyscope::init();

    PointSet *ps = new PointSet();
    ps->readPly("../assets/gaussian_spike_norm.ply");

    InputOctree *octree = generateInputOctree(5, ps);

    auto children = octree->getChildren();
    for(int i = 0; i < 8; i++){
      drawCube(std::to_string(i), children[i]->getMin(), children[i]->getMax());
    }

    pointSetToPolyscope("gaussian", ps);


    polyscope::show();

    return 0;
}
