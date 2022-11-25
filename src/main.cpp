#include <iostream>
#include <string>
#include <filesystem>
#include <functional>

#include "polyscope/messages.h"
#include "polyscope/point_cloud.h"
#include "polyscope/pick.h"
#include "polyscope/polyscope.h"

#include "glm/gtx/string_cast.hpp"

#include "utils.t.hpp"
#include "debug.hpp"
#include "blending.t.hpp"
#include "AlgebraicSphere.t.hpp"

#define MAX_DEPTH 7

namespace fs = std::filesystem;

std::string pathToDirectory{ "../assets/" };
std::string path{"../assets/Head Sculpture.stl"};
std::vector<string> files;
int current_item = 0;
int depthToShow = 0;
PointSet<point3d> *ps;
Octree<statistics3d, glm::vec3> *octree;
std::array< polyscope::CurveNetwork*, MAX_DEPTH > octreeGraph;

// Displaying it.
polyscope::PointCloud * ps_projected ;
polyscope::PointCloud * pc ;


int projected_points_slider = 0 ;

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

    auto node = octree->getAtDepth(2)[8];
    glm::vec3 q( 10, 10, 10 );
    AlgebraicSphere<glm::vec3, statistics3d> sphere;
    auto stat = cumul_stats( node, &rational_kernel, q);
    sphere.fitSphere( stat, q, &rational_kernel );

    display_sphere( "FIT", sphere.getCenter(), sphere.getRadius() );
    display_sphere( "PointToProject", q, 3.f );
    display_sphere( "ProjectedPoint", sphere.project( q ), 3.f );
    drawCube( "cube", node->getMin(), node->getMax() );

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
    //if(ImGui::SliderInt( "profondeur", &depthToShow, 0, MAX_DEPTH - 1 )) showAtDepth( depthToShow );
    if(ImGui::ListBox("files", &current_item, fileGetter, &files, files.size())){ path = files[current_item]; };
    if(ImGui::Button("load file")) loadPointCloud();
    if(ImGui::SliderInt( "projected_points", &projected_points_slider, 0, 10 )) 
    {
        slide_points(pc, ps_projected, 10, projected_points_slider) ;
    }
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
    //ps_projected = draw_traverseOctree_onePoint(octree, ps);

    /*for( int i = 0; i < MAX_DEPTH; i++ ){
        auto o = octree->getAtDepth( i );
        octreeGraph[ i ] = drawOctree( std::to_string(i), o );
        octreeGraph[ i ]->setEnabled( false );
    }*/

    //pointSetToPolyscope( "point cloud", ps );

    //octreeGraph[0]->setEnabled( true );

    polyscope::view::resetCameraToHomeView();
}
