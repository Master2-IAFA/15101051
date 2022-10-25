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

    InputOctree *octree = generateInputOctree(4, ps);

    drawCube("racine", octree->getMin(), octree->getMax());

    auto o = octree->getAtDepth(4);
    for(int i = 0; i < o.size(); i++){
      drawCube(std::to_string(i), o[i]->getMin(), o[i]->getMax());
    }

    pointSetToPolyscope("gaussian", ps);
    

    polyscope::show();

    return 0;
}
