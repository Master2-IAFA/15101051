#include <iostream>
#include <string>
#include <filesystem>
#include <functional>
#include <omp.h>

#include "polyscope/messages.h"
#include "polyscope/point_cloud.h"
#include "polyscope/pick.h"
#include "polyscope/polyscope.h"

#include "glm/gtx/string_cast.hpp"

#include "utils.t.hpp"

#include "kernels.t.hpp"
#include "AlgebraicSphere.t.hpp"
#include "Octree/InputOctree.t.hpp"
#include "PointSet.t.hpp"
#include "Define.hpp"

#include "Debug/ImguiInputOctreeDebug.t.hpp"
#include "Debug/ImguiFittingDebug.t.hpp"
#include "ImguiDebug.t.hpp"

#define MAX_DEPTH 7

namespace fs = std::filesystem;

ImguiInputOctreeDebug *debug;
ImguiFittingDebug *deebug;

std::string pathToDirectory{ "../assets/" };
std::string path{ "../assets/bunny_30.ply" };
std::vector<string> files;
int current_item = 0;
int depthToShow = 0;

PointSet3D *ps;
InputOctree3D *octree;
std::array< polyscope::CurveNetwork*, MAX_DEPTH > octreeGraph;

polyscope::PointCloud *pc_projected;
polyscope::PointCloud *pc;

int projected_points_slider = 0 ;

void showAtDepth( int depth );
void loadPointCloud();
void callback();

int main () {

    for (const auto & entry : fs::directory_iterator(pathToDirectory)){
        std::string s = entry.path().string();
        files.push_back( s );
    }

    polyscope::init();
    ps = new PointSet<point3d>();
    loadPointCloud();

    debug = new ImguiInputOctreeDebug( std::make_shared<InputOctree3D>( *octree ) );
    deebug = new ImguiFittingDebug( std::make_shared<InputOctree3D>( *octree ) );

    pc = pointSetToPolyscope("point cloud", ps);
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
    ImGui::PushItemWidth( 200 );
    if( ImGui::CollapsingHeader("Octree parameters") ) debug->draw();

    if( ImGui::CollapsingHeader("Fitting") ) deebug->draw();
    //if(ImGui::SliderInt( "profondeur", &depthToShow, 0, MAX_DEPTH - 1 )) showAtDepth( depthToShow );
    // if(ImGui::ListBox("files", &current_item, fileGetter, &files, files.size())){ path = files[current_item]; };
    // if(ImGui::Button("load file")) loadPointCloud();
    // if(ImGui::SliderInt( "projected_points", &projected_points_slider, 0, 10 )) 
    // {
    //     slide_points(pc, ps_projected, 10, projected_points_slider) ;
    // }
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

    octree = new InputOctree3D( ps );
    octree->fit( 7, 0 );
    std::cout << "...---..." << std::endl;

    polyscope::view::resetCameraToHomeView();
}
