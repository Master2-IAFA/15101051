#include <iostream>
#include <string>
#include <filesystem>

#include "polyscope/messages.h"
#include "polyscope/point_cloud.h"
#include "polyscope/pick.h"
#include "polyscope/polyscope.h"

#include "glm/gtx/string_cast.hpp"

#include "utils.hpp"
#include "debug.hpp"
#include "blending.hpp"

#define MAX_DEPTH 7

namespace fs = std::filesystem;

std::string pathToDirectory{ "../assets/" };
std::string path{"../assets/gaussian_spike_norm.ply"};
std::vector<string> files;
int current_item = 0;
int depthToShow = 0;
PointSet *ps;
InputOctree *octree;
std::array< polyscope::CurveNetwork*, MAX_DEPTH > octreeGraph;

void showAtDepth( int depth );
void loadPointCloud();
void callback();

int main(int argc, char **argv){

    for (const auto & entry : fs::directory_iterator(pathToDirectory)){
        std::string s = entry.path();
        files.push_back( s );
    }

    polyscope::init();
    ps = new PointSet();
    ps->readOpenMesh( path );
    loadPointCloud();
    
    polyscope::state::userCallback = callback;
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
    ps->readOpenMesh( std::string( path ) );
    delete octree;
    octree = generateInputOctree( MAX_DEPTH, ps );
    draw_traverseOctree_onePoint(octree);


    for( int i = 0; i < MAX_DEPTH; i++ ){
      auto o = octree->getAtDepth( i );
      octreeGraph[i] = drawOctree( std::to_string(i), o );
      octreeGraph[i]->setEnabled( false );
    }

    pointSetToPolyscope("pointCloud", ps);

    octreeGraph[0]->setEnabled( true );

    polyscope::view::resetCameraToHomeView();
}