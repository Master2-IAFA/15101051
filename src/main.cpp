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

#include "Sphere.hpp"

#define MAX_DEPTH 7

namespace fs = std::filesystem;

std::pair<polyscope::Structure*, size_t> t = std::pair<polyscope::Structure*, size_t> (nullptr, -1) ;

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

    /*for (const auto & entry : fs::directory_iterator(pathToDirectory)){
        std::string s = entry.path();
        files.push_back( s );
    }*/
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

  polyscope::view::resetCameraToHomeView();

  if(polyscope::pick::haveSelection()){
      if (t != polyscope::pick::getSelection())
      {
        t = polyscope::pick::getSelection();
        std::cout << "have selection !" << t.first << t.second << "\n" ;
        //t.first->remove() ;
         std::vector<glm::vec3> newPositions ;
         float mouseX = 0.0f ;
         mouseX = ImGui::GetMousePos().x - ImGui::GetCursorScreenPos().x - ImGui::GetScrollX();
         std::cout << "mouseX= " << mouseX ;
         //t.first->updatePointPositions(newPositions);
      }

  }
}

void showAtDepth( int depth ){
  for( int i = 0; i < MAX_DEPTH; i++ ){
    octreeGraph[i]->setEnabled( false );
  }
  octreeGraph[ depth ]->setEnabled( true );
}

void loadPointCloud(){
    ps->readPly( std::string( path ) );
    delete octree;
    octree = generateInputOctree( MAX_DEPTH, ps );

    //debug Lou fit sphere
    auto o = octree->getAtDepth( 2 );
    Sphere * sphere = new Sphere() ;
    sphere->fit_sphere_on_node(o[0], ps, glm::vec3(1.0, 1.0, 1.0));
    //  fit_sphere_on_node(o[2], ps, glm::vec3(1.0, 1.0, 1.0));
    /*for(int i = 0; i < o.size(); i++){
        fit_sphere_on_node(o[i]);
    }*/

  /*  for( int i = 0; i < MAX_DEPTH; i++ ){
      auto o = octree->getAtDepth( i );
      octreeGraph[i] = drawOctree( std::to_string(i), o );
      octreeGraph[i]->setEnabled( false );
    }*/

    pointSetToPolyscope("pointCloud", ps);

    //octreeGraph[0]->setEnabled( true );

}
