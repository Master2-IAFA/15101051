#include "utils.hpp"

template<typename T>
inline T norm (std::vector<T> v) {
    T n = 0;
    for ( size_t i = 0 ; i < v.size() ; ++i ) {
        n += v[i] * v[i];
    }

    return n;
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

template<typename statistics, typename point>
InputOctree *generateInputOctree( int max_depth, PointSet<statistics> *pc ) {
    std::pair<point, point> aabb = pc->getBoundingBox();
    InputOctree *octree = new InputOctree(0, aabb.first.pos, aabb.second.pos);
    std::vector<point> points = pc->getPoints();
    fitInputOctree( max_depth, octree, &points );

    return octree;
}

template<typename statistics, typename point>
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

template<typename statistics, typename point>
void statisticsAdd(statistics *stat, point p) {
    stat->area += 1;
    stat->norm += norm(p.pos);
    stat->pdn += dot(p.pos, p.norm);
    for ( int i = 0 ; i < p.pos.size() ; ++i ) {
        stat->normal[i] += p.norm[i];
        stat->position[i] += p.pos[i];
    }
}
