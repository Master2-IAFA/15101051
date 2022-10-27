#include "utils.hpp"

template<typename T>
inline T norm (std::vector<T> v) {
    T n = 0;
    for ( size_t i = 0 ; i < v.size() ; ++i ) {
        n += v[i] * v[i];
    }

    return sqrt(n);
}

template<typename T>
inline T dot (std::vector<T> a, std::vector<T> b) {
    if (a.size() != b.size()) {
        std::cout << "incoherent vectors' size while computing dot" << std::endl;
    }

    T p = 0;
    for ( size_t i = 0 ; i < a.size() ; ++i ) {
        p += a[i] * b[i];
    }

    return p;
}

InputOctree *generateInputOctree( int max_depth, PointSet *pc ) {
    std::pair<point, point> aabb = pc->getBoundingBox();
    InputOctree *octree = new InputOctree(0, aabb.first.pos, aabb.second.pos, aabb.first.pos.size());
    std::vector<point> points = pc->getPoints();
    fitInputOctree( max_depth, octree, &points );

    return octree;
}

void fitInputOctree( int max_depth, InputOctree *octree, std::vector<point> *points ) {
    if ( max_depth == 0 ) return;

    octree->subDivide();

    auto children = octree->getChildren();
    bool hasPoint = false;

    for ( int i = 0; i < int(pow(2, octree->getDim())); i++ ) {
        std::vector<point> children_points;
        children_points.clear();
        statistics stat;
        stat.normal.clear();
        stat.position.clear();
        for ( int k = 0 ; k < octree->getDim() ; ++k ) {
            stat.position.emplace_back(0);
            stat.normal.emplace_back(0);
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

void statisticsAdd(statistics *stat, point point) {
    stat->area += 1;
    stat->norm += norm(point.pos);
    stat->normal[0] += point.norm[0];
    stat->normal[1] += point.norm[1];
    stat->normal[2] += point.norm[2];
    stat->pdn += dot(point.pos, point.norm);
    stat->position[0] += point.pos[0];
    stat->position[1] += point.pos[1];
    stat->position[2] += point.pos[2];
}
