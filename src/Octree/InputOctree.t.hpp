#pragma once

#include "InputOctree.hpp"

template< class VecType, class StatType, class PointType >
void InputOctree< VecType, StatType, PointType>::fit( int max_depth, int max_points ){

    if( m_pointSet == nullptr ) return;

    std::vector< PointType > points = m_pointSet->getPoints();
    StatType root_stats;
    for (auto p : points){
        statisticsAdd( &root_stats, p );
    }
    this->setData( root_stats );

    this->subDivide();

    // Draw the protection sphere of the root node

    recursiveFit( max_depth, &points );

}

void func( float x, float &val ){

    if( x >= 1.0 ){
        val = 1.0f;
    }else{
        val = std::exp( - std::exp( 1.0f / ( x - 1.0f ) ) / ( x * x ) );
    }

}


template< class VecType, class StatType, class PointType >
StatType InputOctree< VecType, StatType, PointType>::getBlendedStat( PointType point, std::function< float( VecType&, VecType& ) > kernel){

    if( this->m_data.area == 0 ){
        return this->m_data;
    }

    StatType stat;
    auto t = this->m_data.position / VecType(this->m_data.area);
    float w = kernel(t, point.pos);

    //if leaf
    if( !this->hasChildren() ) {

        if (this->m_data.area > 0) {
            for( auto p : m_points ){
                StatType pStat;
                statisticsAdd( &pStat, p );
                pStat = weighted_statistics( pStat, kernel( p.pos, point.pos ) );
                stat = sum_statistics( stat, pStat );
            }
        }
        return stat;

    //q is sufficiently far from the node
    }else if( signedDistanceToProtectionSphere( point.pos ) >= 0 ){

        stat = weighted_statistics( this->m_data, w );
        return stat;

    }else{

        float distanceToNode = fabs(signedDistanceToProtectionSphere(point.pos));

        for (auto child: this->m_children) {
            float distanceToChild = fabs(child->signedDistanceToProtectionSphere(point.pos));
            float wChild = distanceToChild / (distanceToChild - distanceToNode);
            float Yu;
            func(wChild, Yu);
            float valParent = 1.0f - Yu;

            auto leftStat = weighted_statistics(child->getBlendedStat(point, kernel), 1.0 - Yu);

            float areaW = child->getData().area / this->m_data.area;
            auto rightStat = weighted_statistics(this->getData(), areaW * Yu * w);

            stat = sum_statistics(stat, leftStat);
            stat = sum_statistics(stat, rightStat);
        }

        return stat;
    }

}

template< class VecType, class StatType, class PointType >
float InputOctree< VecType, StatType, PointType>::signedDistanceToProtectionSphere( VecType point ){
    VecType min = this->getMin();
    VecType max = this->getMax();
    float radius_protectionSphere = (glm::distance(min, max) / 2.0) * (*m_protectionSphere);
    VecType mid = ( min + max ) / VecType ( 2.0 );
    return glm::distance(mid, point) - radius_protectionSphere;
}



template< class VecType, class StatType, class PointType >
bool InputOctree< VecType, StatType, PointType>::isInProtectionSphere( VecType point ){
    return signedDistanceToProtectionSphere(point) <= 0;
}

template< class VecType, class StatType, class PointType >
void InputOctree< VecType, StatType, PointType>::recursiveFit( int depth, std::vector<PointType> *points ){

    if ( depth == 0 ){
        for (auto p : *points)
            m_points.push_back( p );
        this->getChildren()[0] = nullptr;
        return;
    }

    this->subDivide();

    auto children = this->getChildren();
    bool hasPoint = false;

    for ( int i = 0; i < int(pow(2, this->getDim())); i++ ) {
        std::vector<PointType> children_points;
        StatType stat;

        hasPoint = false;

        for ( int j = 0; j < points->size(); ++j ) {
            if ( children[i]->isPointIn( points->at(j).pos ) ) {
                children_points.emplace_back( points->at(j) );
                statisticsAdd( &stat, points->at(j) );
                hasPoint = true;
            }
        }

        if (hasPoint) {
        //     // TEST EN METTANT UNE DENSITE AUX NOEUDS;
        //     InputOctree< VecType, StatType, PointType> * father_test = this;
        //     while (father_test->getDepth() != 0) 
        //         father_test = father_test->m_father;

        //     stat.area /= father_test->getData().area;
        //     // FIN TEST EN METTANT UNE DENSITE AUX NOEUDS;

            children[i]->setData( stat );
            children[i]->recursiveFit( depth - 1, &children_points );
        }

    }
}
/**
 * @brief EQUATION 7 DANS LE PAPIER
 * 
 */
template< class VecType, class StatType, class PointType >
float InputOctree< VecType, StatType, PointType>::gamma_maj (InputOctree<VecType, StatType, PointType> *child, VecType q){
    float distance_to_node = this->signedDistanceToProtectionSphere( q ); //signedDistanceToSphere(node, q);
    float distance_to_child = child->signedDistanceToProtectionSphere( q ); //signedDistanceToSphere(child, q);

    if (distance_to_node <= 0 && distance_to_child <= 0) {
        return 0;
    }
    if (distance_to_node >= 0 && distance_to_child >= 0) {
        return 1;
    }
    float u = (distance_to_child / (distance_to_child - distance_to_node));
    return exp(-exp(1.0/(u-1.0)) / pow(u,2));
}