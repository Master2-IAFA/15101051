#pragma once

#include <iostream>
#include <omp.h>
#include <string>
#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "polyscope/curve_network.h"
#include "utils.t.hpp"
#include "PointSet.t.hpp"
#include "Octree/BaseOctree.t.hpp"
#include "Octree/Octree.t.hpp"

template<class VecType, class PointType>
polyscope::PointCloud* pointSetToPolyscope(std::string name, PointSet<PointType> *ps);

/** given min and max of cube, draws cube on polyscope
 * @param [in] name : name of polyscope window
 * @param [in] min : min point of cube
 * @param [in] max : max point of cube
 */
template<class VecType>
void drawCube(std::string name, VecType min, VecType max);

/** displays sphere on polyscope according to its radius and center */
template<class VecType>
void display_sphere( std::string name, VecType center, float radius) ;

template<typename Data, typename VecType, typename OctreeType>
polyscope::CurveNetwork* drawOctree(std::string name, std::vector<BaseOctree<Data, VecType, OctreeType>*> octree);

/** function to vizualise points sliding accross their translation */
void slide_points(polyscope::PointCloud *, polyscope::PointCloud * pc_final, int nb_slider_max, int nb_slider) ;

/** It displays the traversed nodes of the octree.
 * @author Léo 
 * @param [in] name name of the displayed curve.
 * @param [in] q point that we want to use.
 */
template< class VecType, class StatType, class PointType >
void draw_traversed_octree (std::shared_ptr< InputOctree<VecType, StatType, PointType > > oct, VecType q, std::string name);
