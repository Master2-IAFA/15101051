#include <iostream>

#include "polyscope/messages.h"
#include "polyscope/point_cloud.h"
#include "polyscope/pick.h"
#include "polyscope/polyscope.h"

#include "glm/gtx/string_cast.hpp"

#include "utils.hpp"
#include "debug.hpp"

#define MAX_DEPTH 7

int depthToShow = 0;
std::array< polyscope::CurveNetwork*, MAX_DEPTH > octreeGraph;

void showAtDepth( int depth );

void callback(){
  ImGui::PushItemWidth( 100 );
  if(ImGui::SliderInt( "profondeur", &depthToShow, 0, MAX_DEPTH - 1 )) showAtDepth( depthToShow );

}




int main(int argc, char **argv){

    polyscope::init();

    PointSet *ps = new PointSet();
    ps->readOpenMesh("../assets/bunny2.ply");

    InputOctree *octree = generateInputOctree( MAX_DEPTH, ps );

    for( int i = 0; i < MAX_DEPTH; i++ ){
      auto o = octree->getAtDepth( i );
      octreeGraph[i] = drawOctree( std::to_string(i), o );
      octreeGraph[i]->setEnabled( false );
    }

    octreeGraph[0]->setEnabled( true );



    

    pointSetToPolyscope("gaussian", ps);
    polyscope::state::userCallback = callback;
    polyscope::show();

    return 0;
}

void showAtDepth( int depth ){
  for( int i = 0; i < MAX_DEPTH; i++ ){
    octreeGraph[i]->setEnabled( false );
  }
  octreeGraph[ depth ]->setEnabled( true );
}