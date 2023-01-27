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
#include "Debug/ImguiFileSelection.t.hpp"
#include "ImguiDebug.t.hpp"

#define MAX_DEPTH 7

namespace fs = std::filesystem;

ImguiInputOctreeDebug<glm::vec2, statistics2d, point2d> *debug;
ImguiFittingDebug<glm::vec2, statistics2d, point2d> *deebug;
ImguiFileSelection<glm::vec2, statistics2d, point2d> *imguiFileSelection;

std::string pathToDirectory{ "../assets/" };
std::string path{ "../assets/gaussian.ply" };
std::vector<string> files;
int current_item = 0;
int depthToShow = 0;

PointSet2D *ps;
InputOctree2D *octree;
std::array< polyscope::CurveNetwork*, MAX_DEPTH > octreeGraph;

polyscope::PointCloud *pc_projected;
polyscope::PointCloud *pc;

int projected_points_slider = 0 ;

void showAtDepth( int depth );
void loadPointCloud();
void callback();

int main () {
    polyscope::init();
    polyscope::view::style = polyscope::view::NavigateStyle::Planar;

    ps = new PointSet<point2d>();
    loadPointCloud();

    std::cout << "--1--" << std::endl;
    auto octreePtr = std::make_shared<InputOctree2D>( *octree );

    std::cout << "--2--" << std::endl;
    imguiFileSelection = new ImguiFileSelection<glm::vec2, statistics2d, point2d>( ps, octreePtr, std::string("../assets/") );

    std::cout << "--2--" << std::endl;
    debug = new ImguiInputOctreeDebug( octreePtr );

    std::cout << "--4--" << std::endl;
    deebug = new ImguiFittingDebug( octreePtr );

    std::cout << "--5--" << std::endl;
    pc = pointSetToPolyscope<glm::vec2, point2d>("point cloud", ps);
    polyscope::state::userCallback = callback;
    polyscope::show();

    delete ps;
    delete octree;

    return 0;
}

void callback(){
    ImGui::PushItemWidth( 200 );

    if( ImGui::CollapsingHeader("file selection") ) imguiFileSelection->draw();

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

    octree = new InputOctree2D( ps );
    octree->fit( 7, 0 );
    std::cout << "...---..." << std::endl;

    polyscope::view::resetCameraToHomeView();
}
