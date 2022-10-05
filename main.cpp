
#include "polyscope/messages.h"
#include "polyscope/point_cloud.h"
#include "polyscope/pick.h"
#include "polyscope/polyscope.h"

#include "Octree.hpp"

void reconstruct(){

}


float kernelSize = 1.0f;
polyscope::PointCloud* psCloud;
std::vector<glm::vec3> points;
std::vector<glm::vec3> colors;
int prev = -1;

void callback() {

    ImGui::PushItemWidth(100);
    ImGui::Text("custom window");

    ImGui::SliderFloat("Kernel size", &kernelSize, 0.0f, 1.0f, "%.3f");
    ImGui::Button("Surface");

    ImGui::PopItemWidth();

    if(polyscope::pick::haveSelection()){
        std::pair<polyscope::Structure*, size_t> t = polyscope::pick::getSelection();
        if( t.second != prev ){
            std::cout << t.second << std::endl;
            prev = t.second;
            colors[prev] = glm::vec3( 1, 0, 0 );
            polyscope::getPointCloud("really great points")->addColorQuantity("random color", colors);
        }
        
    }
}



int main(int argc, char **argv){
    
    polyscope::init();
    std::vector<glm::vec3> points;

    // generate points
    for(float i = 0; i < 50; i++){
        for( float j = 0; j < 50; j++){
            float x = (i - 25) / 25.0f;
            float y = (j - 25) / 25.0f;
            points.push_back( glm::vec3( i/50.0f, exp( -(x*x) - (y*y)) , j/50.0f));
            colors.push_back( glm::vec3(0.0f, 0.0f, 0.0f) );
        }
    }

    // visualize!
    psCloud = polyscope::registerPointCloud("really great points", points);
    

    // set some options
    psCloud->setPointRadius(0.02);
    psCloud->setPointRenderMode(polyscope::PointRenderMode::Sphere);
    //psCloud->buildPickUI(0);

    

    // visualize
    auto e = polyscope::getPointCloud("really great points")->addColorQuantity("random color", colors);
    e->setEnabled(true);

    polyscope::state::userCallback = callback;

    // show
    polyscope::show();

    return 0;
}