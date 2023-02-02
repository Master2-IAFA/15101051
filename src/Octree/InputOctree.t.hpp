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

    recursiveFit( max_depth, &points );

}


template< class VecType, class StatType, class PointType >
StatType InputOctree< VecType, StatType, PointType>::getBlendedStat( PointType point, std::function< float( VecType&, VecType& ) > kernel){
    StatType father_stats = this->getData();
    double sigma_n = father_stats.area;
    double sigma_nu;

    // If node is a leaf :
    if ( ! this->hasChildren() ){
        float weight = 0.0f;
        // Accumulate statistics over points in the leaf.
        for (VecType p : this->getPoints()){
            weight += kernel(point.pos, p);
        }
        return weighted_statistics(father_stats, weight);
    }
    // float sigma_n_toFloat = round(sigma_n * pow(10, 7)) / pow(10, 7);
    sigma_n = ( sigma_n == 0? 1 : sigma_n );
    VecType averagePosition = father_stats.position / ((float)sigma_n);
    float weight_averagePos = kernel (point.pos, averagePosition);

    // Case q isn't in the node
    if (! this->isInProtectionSphere(point.pos) ){
        return weighted_statistics(father_stats, weight_averagePos);
    }
    // Let's blend statistics between n and its children
    StatType node_stats;
    auto children = this->getChildren();

    for (auto child : children){
        double weight_area = (child->getData().area)/sigma_n;
        float gamma = this->gamma_maj( child, point.pos );
        float weight = weight_averagePos * weight_area * gamma;
        StatType child_stats = child->getBlendedStat(point, kernel);
        node_stats = sum_statistics(node_stats, weighted_statistics(child_stats, (1-gamma)));
        node_stats = sum_statistics(node_stats, weighted_statistics(father_stats, weight));
    }
    // std::cout << "==========================" << std::endl;
    // display_statistics(node_stats);
    return node_stats;
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
    return (!(signedDistanceToProtectionSphere ( point ) > 0));
}

template< class VecType, class StatType, class PointType >
void InputOctree< VecType, StatType, PointType>::recursiveFit( int depth, std::vector<PointType> *points ){

    if ( depth == 0 ){
        std::vector<VecType> vecTypePoints;
        for (auto p : *points)
            vecTypePoints.emplace_back(p.pos);
        this->setPoints(vecTypePoints) ;
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