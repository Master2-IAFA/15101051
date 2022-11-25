#pragma once
#include "utils.hpp"


template<typename statistics, typename point, typename VecType>
Octree<statistics, VecType>* generateInputOctree( int max_depth, PointSet<point> *pc ) {
    std::pair<point, point> aabb = pc->getBoundingBox();
    Octree<statistics, VecType>* octree = new Octree<statistics, VecType>(0, aabb.first.pos, aabb.second.pos);
    std::vector<point> points = pc->getPoints();

    statistics root_stats;
    for (auto p : points)
        statisticsAdd( &root_stats, p );
    octree->setData ( root_stats );

    fitInputOctree<statistics, point, VecType>( max_depth, octree, &points );
    return octree;
}

template<typename statistics, typename point, typename VecType>
void fitInputOctree( int max_depth, Octree<statistics, VecType>* octree, std::vector<point> *points ) {
    if ( max_depth == 0 ){
        std::vector<VecType> vecTypePoints;
        for (auto x : *points)
            vecTypePoints.emplace_back(x.pos);
        octree->setPoints(vecTypePoints) ;
        octree->getChildren()[0] = nullptr;
        return;
    }

    octree->subDivide();

    auto children = octree->getChildren();
    bool hasPoint = false;

    for ( int i = 0; i < int(pow(2, octree->getDim())); i++ ) {
        std::vector<point> children_points;
        statistics stat;

        hasPoint = false;

        for ( int j = 0; j < points->size(); ++j ) {
            if ( children[i]->isPointIn( points->at(j).pos ) ) {
                children_points.emplace_back( points->at(j) );
                statisticsAdd( &stat, points->at(j) );
                hasPoint = true;
            }
        }

        if (hasPoint) {
            children[i]->setData( stat );
            fitInputOctree( max_depth - 1, children[i], &children_points );
        }
    }
}

/**
 * @brief This function updates the given stats with the information of the given point.
 *
 */
template<typename statistics, typename point>
void statisticsAdd(statistics *stat, point p) {
    // sigma
    stat->area += 1;
    // P_beta
    stat->norm += glm::pow(glm::l2Norm(p.pos),2);
    // Pn_beta
    stat->pdn += glm::dot(p.pos, p.norm);
    // P and N _alpha
    for ( int i = 0 ; i < p.pos.length() ; ++i ) {
        stat->normal[i] += p.norm[i];
        stat->position[i] += p.pos[i];
    }
}

/**
 * @brief this function prints the informations contained in this stats.
 */
template <typename statistics> 
void display_statistics (statistics stats){
    if (stats.position.length() == 2) {
        std::cout << "Position : (" << stats.position[0] << ", " << stats.position[1] << ")." << std::endl;
        std::cout << "Normal : (" << stats.normal[0] << ", " << stats.normal[1] << ")." << std::endl;
    }
    else {
        std::cout << "Position : (" << stats.position[0] << ", " << stats.position[1] << ", " << stats.position[2] << ")." << std::endl;
        std::cout << "Normal : (" << stats.normal[0] << ", " << stats.normal[1] << ", " << stats.normal[2] << ")." << std::endl;
    }
    std::cout << "Norm : " << stats.norm << std::endl;
    std::cout << "Area : " << stats.area << std::endl;
    std::cout << "Pdn : " << stats.pdn << std::endl;
}