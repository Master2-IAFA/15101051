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

bool render2d = false;
bool render3d = true;

ImguiInputOctreeDebug<glm::vec3, statistics3d, point3d> *octreeGui;
ImguiFittingDebug<glm::vec3, statistics3d, point3d> *fittingGui;
ImguiFileSelection<glm::vec3, statistics3d, point3d> *imguiFileSelection;

ImguiInputOctreeDebug<glm::vec2, statistics2d, point2d> *octreeGui2d;
ImguiFittingDebug<glm::vec2, statistics2d, point2d> *fittingGui2d;
ImguiFileSelection<glm::vec2, statistics2d, point2d> *imguiFileSelection2d;

PointSet3D *ps;
InputOctree3D *octree;

PointSet2D *ps2d;
InputOctree2D *quadtree;

std::string pathToDirectory{ "../assets/" };
std::string path{ "../assets/gaussian.ply" };
std::vector<string> files;
int current_item = 0;
int depthToShow = 0;

std::array< polyscope::CurveNetwork*, MAX_DEPTH > octreeGraph;

polyscope::PointCloud *pc_projected;
polyscope::PointCloud *pc;

int projected_points_slider = 0 ;

void showAtDepth( int depth );
void callback();
void generate3DPointCloud();
void generate2DPointCloud();

int main () {
    polyscope::init();

    ps = new PointSet3D();
    ps2d = new PointSet2D();

    generate2DPointCloud();
    generate3DPointCloud();

    polyscope::state::userCallback = callback;
    polyscope::show();

    delete octree;
    delete ps;

    delete quadtree;
    delete ps2d;

    return 0;
}

void callback(){
    ImGui::PushItemWidth( 200 );

    if (ImGui::RadioButton("3D", render3d)) {
        render3d = true;
        render2d = false;

        //free navigation camera
        polyscope::view::style = polyscope::view::NavigateStyle::Free;
        generate3DPointCloud();
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("2D", render2d)) {
        render3d = false;
        render2d = true;

        //2D planar camera
        polyscope::view::style = polyscope::view::NavigateStyle::Planar;
        generate2DPointCloud();
    }

    if( ImGui::CollapsingHeader("file selection") ) {
        if (render3d) imguiFileSelection->draw();
        else imguiFileSelection2d->draw();
    }

    if( ImGui::CollapsingHeader("Octree parameters") ) {
        if (render3d) octreeGui->draw();
        else octreeGui2d->draw();
    }

    if( ImGui::CollapsingHeader("Fitting") ) {
        if (render3d) fittingGui->draw();
        else fittingGui2d->draw();
    }
}

void showAtDepth( int depth ){
    for( int i = 0; i < MAX_DEPTH; i++ ){
        octreeGraph[i]->setEnabled( false );
    }
    octreeGraph[ depth ]->setEnabled( true );
}

void generate3DPointCloud() {
    //create and read 3D point cloud from file path
    ps->readOpenMesh( std::string( path ) );

    //generate octree for newly read point cloud
    delete octree;
    octree = new InputOctree3D( ps );
    octree->fit( 7, 0 );

    //init interface component pointers
    auto octreePtr = std::make_shared<InputOctree3D>( *octree );
    imguiFileSelection = new ImguiFileSelection<glm::vec3, statistics3d, point3d>( ps, octreePtr, std::string("../assets/") );
    octreeGui = new ImguiInputOctreeDebug( octreePtr );
    fittingGui = new ImguiFittingDebug( octreePtr );

    //render point cloud in polyscope
    pc = pointSetToPolyscope<glm::vec3, point3d>("point cloud", ps);

    //set camera to point cloud front
    polyscope::view::resetCameraToHomeView();
}

void generate2DPointCloud() {
    //generating sampeled gaussian instead of loading from file
    *ps2d = generate2dGaussian();

    delete quadtree;
    quadtree = new InputOctree2D( ps2d );
    quadtree->fit( 7, 0 );

    auto quadtreePtr = std::make_shared<InputOctree2D>( *quadtree );
    imguiFileSelection2d = new ImguiFileSelection<glm::vec2, statistics2d, point2d>( ps2d, quadtreePtr, std::string("../assets/") );
    octreeGui2d = new ImguiInputOctreeDebug( quadtreePtr );
    fittingGui2d = new ImguiFittingDebug( quadtreePtr );

    pc = pointSetToPolyscope<glm::vec2, point2d>("point cloud", ps2d);
    polyscope::view::resetCameraToHomeView();
}
