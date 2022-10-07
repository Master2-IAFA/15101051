
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
    drawCube(glm::vec3(0.0, 0.0, 0.0), glm::vec3(3.0, 3.0, 3.0));

    return 0;
}
