#include <iostream>
#include <string>
#include <filesystem>

#include "polyscope/messages.h"
#include "polyscope/point_cloud.h"
#include "polyscope/pick.h"
#include "polyscope/polyscope.h"

#include "glm/gtx/string_cast.hpp"

#include "utils.t.hpp"
#include "debug.hpp"

#define MAX_DEPTH 3

namespace fs = std::filesystem;

std::string pathToDirectory{ "../assets/" };
std::string path{"../assets/bunny2.ply"};
std::vector<string> files;
int current_item = 0;
int depthToShow = 0;
PointSet<point3d> *ps;
PointSet<point2d> ps2d;
Octree<statistics3d, glm::vec3> *octree;
Octree<statistics2d, glm::vec2> *quad;
std::array< polyscope::CurveNetwork*, MAX_DEPTH > octreeGraph;

void showAtDepth( int depth );
void loadPointCloud();
void callback();

int main () {
//    for (const auto & entry : fs::directory_iterator(pathToDirectory)){
//        std::string s = entry.path();
//        files.push_back( s );
//    }

    polyscope::init();
    ps2d = generate2dGaussian();
    quad = generateInputOctree<statistics2d, point2d, glm::vec2>( MAX_DEPTH, &ps2d );

    pointSet2dToPolyscope("point cloud", &ps2d);
    for( int i = 0; i < MAX_DEPTH; i++ ) {
        auto o = quad->getAtDepth( i );
        std::cout << "2" << std::endl;
        octreeGraph[i] = drawQuadtree( std::to_string(i), o );
        std::cout << "3" << std::endl;
        octreeGraph[i]->setEnabled( false );
        std::cout << "4" << std::endl;
    }
    //drawSquare("charabia", glm::vec2(1, 0), glm::vec2(0, 1));

    octreeGraph[0]->setEnabled( true );

    //loadPointCloud();

    polyscope::state::userCallback = callback;
    std::cout << "4" << std::endl;

    polyscope::show();

    delete ps;
    delete octree;
    delete quad;

    return 0;
}

bool fileGetter(void *data, int index, const char** output)
{
    std::vector<string>* vec = (std::vector<string>*)data;
    string &s = vec->at(index);
    *output = s.c_str(); // not very safe
    return true;
}

void callback(){
    ImGui::PushItemWidth( 100 );
    if(ImGui::SliderInt( "profondeur", &depthToShow, 0, MAX_DEPTH - 1 )) showAtDepth( depthToShow );
   // if(ImGui::ListBox("files", &current_item, fileGetter, &files, files.size())){ path = files[current_item]; };
   // if(ImGui::Button("load file")) loadPointCloud();
}

void showAtDepth( int depth ){
    for( int i = 0; i < MAX_DEPTH; i++ ){
        octreeGraph[i]->setEnabled( false );
    }
    octreeGraph[ depth ]->setEnabled( true );
}

void loadPointCloud(){
    ps->readOpenMesh( std::string( path ) );
    delete octree;

    octree = generateInputOctree<statistics3d, point3d, glm::vec3>( MAX_DEPTH, ps );

    for( int i = 0; i < MAX_DEPTH; i++ ){
        auto o = octree->getAtDepth( i );
        octreeGraph[i] = drawOctree( std::to_string(i), o );
        octreeGraph[i]->setEnabled( false );
    }

    pointSetToPolyscope("point cloud", ps);

    octreeGraph[0]->setEnabled( true );

    polyscope::view::resetCameraToHomeView();
}
