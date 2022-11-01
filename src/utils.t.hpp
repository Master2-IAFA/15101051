#pragma once

#include "utils.hpp"

template<typename statistics, typename point, typename VecType>
Octree<statistics, VecType>* generateInputOctree( int max_depth, PointSet<point> *pc ) {
    std::pair<point, point> aabb = pc->getBoundingBox();
    Octree<statistics, VecType>* octree = new Octree<statistics, VecType>(0, aabb.first.pos, aabb.second.pos);
    std::vector<point> points = pc->getPoints();

    std::cout << "4" << std::endl;
    fitInputOctree<statistics, point, VecType>( max_depth, octree, &points );

    std::cout << "5" << std::endl;
    return octree;
}

template<typename statistics, typename point, typename VecType>
void fitInputOctree( int max_depth, Octree<statistics, VecType>* octree, std::vector<point> *points ) {
    if ( max_depth == 0 ) return;

    octree->subDivide();

    auto children = octree->getChildren();
    bool hasPoint = false;

    for ( int i = 0; i < int(pow(2, octree->getDim())); i++ ) {
        std::vector<point> children_points;
        children_points.clear();
        statistics stat;
        for ( int k = 0 ; k < octree->getDim() ; ++k ) {
            stat.position[i] = 0;
            stat.normal[i] = 0;
        }

        hasPoint = false;

        for ( int j = 0; j < points->size(); ++j ) {
            if ( children[i]->isPointIn( points->at(j).pos ) ) {
                children_points.push_back( points->at(j) );
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

template<typename statistics, typename point>
void statisticsAdd(statistics *stat, point p) {
    std::cout << "..";
    stat->area += 1;
    stat->norm += glm::l2Norm(p.pos) * glm::l2Norm(p.pos);
    stat->pdn += glm::dot(p.pos, p.norm);
    for ( int i = 0 ; i < p.pos.length() ; ++i ) {
        stat->normal[i] += p.norm[i];
        stat->position[i] += p.pos[i];
    }
}
