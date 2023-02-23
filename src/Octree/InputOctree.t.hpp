#pragma once

#include "../utils.t.hpp"
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
/** equation 7 in the paper */
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

template< class VecType, class StatType, class PointType >
void InputOctree< VecType, StatType, PointType>::getTraversedNodes (VecType& q, std::vector<std::array<int, 2>>* edges, std::vector<VecType>* nodes) {
    if (!this->hasChildren())
        return;
    auto children = this->getChildren();

    auto cube = build_cube_from_minmax( this->getMin(), this->getMax());
    int vecLength(this->getMin().length());

    int nodes_size = nodes->size();

    for(int j = 0; j < pow (2, vecLength); j++)
        nodes->push_back( cube[j] );

    for (int j = 0;j < pow(2, vecLength);++j) {
        for (int k = j + 1;k < pow(2, vecLength);++k) {
            if (bitDiff(k, j) == 1) {
                edges->push_back({j + nodes_size, k + nodes_size});
            }
        }
    }

    for (int i = 0; i < pow(2, vecLength); i++){
        if (children[i]->isInProtectionSphere( q))
            children[i]->getTraversedNodes(q, edges, nodes);
    }
}