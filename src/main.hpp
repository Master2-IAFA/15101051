#pragma once
#include "utils.t.hpp"
#include "debug.hpp"
#include "blending.t.hpp"

#include <iostream>
#include <string>
#include <filesystem>

#include "polyscope/messages.h"
#include "polyscope/point_cloud.h"
#include "polyscope/pick.h"
#include "polyscope/polyscope.h"

#include "glm/gtx/string_cast.hpp"

#define MAX_DEPTH 7

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
