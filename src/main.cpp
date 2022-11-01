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

#define MAX_DEPTH 7

namespace fs = std::filesystem;

std::string pathToDirectory{ "../assets/" };
std::string path{"../assets/gaussian_spike_norm.ply"};
std::vector<string> files;
int current_item = 0;
int depthToShow = 0;
PointSet<point3d> *ps;
Octree<statistics3d, glm::vec3> *octree;
std::array< polyscope::CurveNetwork*, MAX_DEPTH > octreeGraph;

void showAtDepth( int depth );
void loadPointCloud();
void callback();

int main () {
    for (const auto & entry : fs::directory_iterator(pathToDirectory)){
        std::string s = entry.path();
        files.push_back( s );
    }

    polyscope::init();
    ps = new PointSet<point3d>();

    loadPointCloud();

    polyscope::state::userCallback = callback;
    pointSetToPolyscope("", ps);

    polyscope::show();

    delete ps;
    delete octree;

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
    if(ImGui::ListBox("files", &current_item, fileGetter, &files, files.size())){ path = files[current_item]; };
    if(ImGui::Button("load file")) loadPointCloud();
}

void showAtDepth( int depth ){
    for( int i = 0; i < MAX_DEPTH; i++ ){
        octreeGraph[i]->setEnabled( false );
    }
    octreeGraph[ depth ]->setEnabled( true );
}

void loadPointCloud(){
    std::cout << "1" << std::endl;
    ps->readOpenMesh( std::string( path ) );
    std::cout << "2" << std::endl;
    delete octree;

    std::cout << "3" << std::endl;
    octree = generateInputOctree<statistics3d, point3d, glm::vec3>( MAX_DEPTH, ps );

    std::cout << "4" << std::endl;
    for( int i = 0; i < MAX_DEPTH; i++ ){
        auto o = octree->getAtDepth( i );
        octreeGraph[i] = drawOctree( std::to_string(i), o );
        octreeGraph[i]->setEnabled( false );
    }

    std::cout << "5" << std::endl;
    pointSetToPolyscope("pointCloud", ps);

    std::cout << "6" << std::endl;
    octreeGraph[0]->setEnabled( true );

    std::cout << "7" << std::endl;
    polyscope::view::resetCameraToHomeView();
}
