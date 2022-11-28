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
#include "ImguiDebug.t.hpp"



#define MAX_DEPTH 7

namespace fs = std::filesystem;

ImguiInputOctreeDebug *debug;

std::string pathToDirectory{ "../assets/" };
std::string path{"../assets/Head Sculpture.stl"};
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
    debug = new ImguiInputOctreeDebug( std::make_shared<InputOctree3D>( *octree ));
    
    std::vector<point3d> proj_p( ps->getPoints().size() );
    
    #pragma omp parallel for num_threads( 6 )
    for( int i = 0; i < ps->getPoints().size(); i++ ){
        point3d q = ps->getPoints()[ i ];
        AlgebraicSphere<glm::vec3, statistics3d> sphere;
        auto stat = octree->getBlendedStat( q, &gaussian_mixture );
        sphere.fitSphere( stat, q.pos, &gaussian_mixture );
        auto projected_q = sphere.project( q.pos );
        point3d p;
        p.pos = projected_q;
        p.norm = q.norm;
        proj_p[ i ] = p;
    }

    PointSet<point3d> projected_pointSet( proj_p );

    // auto ppc = pointSetToPolyscope( "projected point cloud", &projected_pointSet);
    // pc = pointSetToPolyscope("point cloud", ps);
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
    debug->draw();
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

    std::cout << "Protection sphere: " << octree->getProtectionSphere() << std::endl;
    std::cout << "Protection sphere: " << octree->getChildren()[0]->getChildren()[0]->getProtectionSphere() << std::endl;
    octree->getChildren()[0]->getChildren()[0]->setProtectionSphere( 1.4 );
    std::cout << "Protection sphere: " << octree->getProtectionSphere() << std::endl;
    std::cout << "Protection sphere: " << octree->getChildren()[0]->getChildren()[0]->getProtectionSphere() << std::endl;

    //InputOctree<glm::vec3, statistics3d, point3d> inputOctree( ps );
    //inputOctree.fit( 7, 0 );
    //std::cout << "fitted" << std::endl;
    //octree = static_cast<BaseOctree*>( inputOctree );

    auto d = octree->getAtDepth( 4 );
    auto di = std::vector< BaseOctree3D< InputOctree3D >* >( d.begin(), d.end() );
    //auto dc = std::vector< BaseOctree<statistics3d, glm::vec3, InputOctree< glm::vec3, statistics3d, point3d> >( d.begin(), d.end() );
    // drawOctree( "octree", di );

    std::cout << octree->getMaxDepth() << std::endl;
    std::cout << di.size() << std::endl;
    std::cout << "min: " << di[2]->getMin().x << " max: " << di[2]->getMin().x  << std::endl;
    

    //octree = generateInputOctree<statistics3d, point3d, glm::vec3>( MAX_DEPTH, ps );
    //ps_projected = draw_traverseOctree_onePoint(octree, ps);

    // for( int i = 0; i < MAX_DEPTH; i++ ){
    //     auto o = inputOctree.getAtDepth( i );
    //     octreeGraph[ i ] = drawOctree( std::to_string(i), o );
    //     octreeGraph[ i ]->setEnabled( false );
    // }

    //pointSetToPolyscope( "point cloud", ps );

    //octreeGraph[0]->setEnabled( true );

    polyscope::view::resetCameraToHomeView();
}
