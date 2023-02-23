#pragma once

#include "debug.hpp"
#include "utils.t.hpp"

template<class VecType, class PointType>
polyscope::PointCloud* pointSetToPolyscope(std::string name, PointSet<PointType> *ps ){
    std::vector<PointType> points = ps->getPoints();

    std::vector<VecType> position ( points.size() );
    std::vector<VecType> normal ( points.size() );

    #pragma omp parallel for
    for (int i = 0; i < points.size(); ++i) {
        position[i] = VecType( points[i].pos );
        normal[i] = VecType( points[i].norm );
    }

    polyscope::PointCloud* pointCloud;
    if (points[0].pos.length() == 3) {
        pointCloud = polyscope::registerPointCloud(name, position);
        pointCloud->addVectorQuantity("normal", normal);
    }
    else {
        pointCloud = polyscope::registerPointCloud2D(name, position);
        pointCloud->addVectorQuantity2D("normal", normal);
    }
    
    return pointCloud ;
}

template< class VecType >
std::vector<VecType> build_cube_from_minmax(VecType min, VecType max) {
    std::vector<VecType> cube ;

    for ( int i = 0 ; i < int(pow( 2, min.length() )) ; ++i ) {
        VecType temp = VecType(0);
        for ( int j = 0 ; j < min.length() ; ++j ) {
            if ( (i / int(pow( 2, j ))) % 2 == 0 ) {
                temp[j] = min[j];
            }
            else {
                temp[j] = max[j];
            }
        }
        cube.emplace_back(temp);
    }

    return cube ;
}

template<class VecType>
void slide_points (polyscope::PointCloud *pc_init, polyscope::PointCloud * pc_final, int nb_slider_max, int nb_slider) {
    //get points pos in pc_init and pc_final
    std::vector<VecType> final_pos_list = pc_final->points ;
    std::vector<VecType> init_pos_list = pc_init->points ;

    std::vector<VecType> newPositions ;

    if (nb_slider_max == nb_slider) {
        pc_init->updatePointPositions(final_pos_list);
    }

    else  {
        for (int i = 0 ; i <  init_pos_list.size() ; ++i) {
            //difference between the 2 pos
            VecType diff = final_pos_list.at(i) - init_pos_list.at(i) ;

            VecType new_pos = (float(nb_slider)/float(nb_slider_max)) * diff ;
            newPositions.push_back(init_pos_list.at(i) + new_pos);
        }
        pc_init->updatePointPositions(newPositions);
    }
    //pc_init->updatePointPositions(final_pos_list);
}

template< class VecType >
void drawCube(std::string name, VecType min, VecType max) {
    //cube nodes
    std::vector<VecType> nodes = build_cube_from_minmax<VecType>(min, max);

    //generate cube edges
    std::vector<std::array<int, 2>> edges ;
    for (int i = 0;i < int(pow(2, min.length()));++i) {
        for (int j = i + 1;j < int(pow(2, min.length()));++j) {
            if (bitDiff(i, j) == 1) {
                edges.push_back({i, j});
            }
        }
    }

    // Add the curve network
    (min.length() == 3)?
    polyscope::registerCurveNetwork(name, nodes, edges):
    polyscope::registerCurveNetwork2D(name, nodes, edges);
}

template<typename Data, typename VecType, typename OctreeType>
polyscope::CurveNetwork* drawOctree(std::string name, std::vector<BaseOctree<Data, VecType, OctreeType> *> octree){
    std::vector<std::array<int, 2>> edges ;
    std::vector<VecType> nodes;

    for(int i = 0; i < octree.size(); i++) {
        //get min/max from each octree cell
        auto min = octree[i]->getMin();
        auto max = octree[i]->getMax();

        //add hypercube coordinates from min/max to the octree
        auto cube = build_cube_from_minmax<VecType>( min , max );
        for(int j = 0; j < int(pow(2, min.length())); j++)
            nodes.push_back( cube[j] );

        //create edges for each hypercube
        for (int j = 0;j < int(pow(2, min.length()));++j) {
            for (int k = j + 1;k < int(pow(2, min.length()));++k) {
                if (bitDiff(k, j) == 1) {
                    edges.push_back({j + int(pow(2, min.length())) * i, k + int(pow(2, min.length())) * i});
                }
            }
        }
    }

    return (octree[0]->getMax().length() == 3)?
                polyscope::registerCurveNetwork(name, nodes, edges):
                polyscope::registerCurveNetwork2D(name, nodes, edges);
}

template< class VecType >
void display_sphere( std::string name, VecType center, float radius) {
    std::vector<VecType> sphere_pos;

    sphere_pos.push_back( center );

    polyscope::PointCloud *pointCloud = (center.length() == 3)?
              polyscope::registerPointCloud( name, sphere_pos ):
              polyscope::registerPointCloud2D( name, sphere_pos );
    pointCloud->setPointRadius(radius, false);
}

template< class VecType, class StatType, class PointType >
void draw_traversed_octree (std::shared_ptr< InputOctree<VecType, StatType, PointType > > oct, VecType q, std::string name){
    std::vector<std::array<int, 2>> edges ;
    std::vector<VecType> nodes;
    oct.get()->getTraversedNodes(q ,&edges, &nodes);
    if (q.length() == 3) 
        polyscope::registerCurveNetwork(name, nodes, edges);
    else
        polyscope::registerCurveNetwork2D(name, nodes, edges);
}
