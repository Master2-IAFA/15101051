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

/**
 * @brief This function takes algebraic parameters and returns the radius of the geometric sphere.
 */
template<typename VecType>
float get_radius(float u0, VecType u123, float u4, VecType c)
{
    float b = u0 / u4;
    auto cTc = glm::dot(c,c);
    float r = sqrt(cTc - b);
    // float radius = sqrt(glm::pow(glm::l2Norm((-0.5f*b)*m_ul), 2) - m_uc*b );
    return max(0, r);
}

/**
 * @brief This function takes algebraic parameters and returns the center of the geometric sphere.
 */
template<typename VecType>
VecType get_center(float u0, VecType u123, float u4)
{   
    float b = - 1.0f / (2 * u4);
    VecType center = b*u123 ;
    return center ;
}

/**
 * @brief This function takes statistics of a node or aggregated statistics and returns the geometric parameters of the sphere.
 * @return : The pair représente in x.first the center of the geometric sphere, and in x.second the radius of it.
 */
template<typename statistics, typename VecType>
std::pair<VecType, float> fit_algebraic_sphere(statistics stat, VecType q, float (*kernel)(VecType&,VecType&)){

    VecType to_Kernel = VecType (stat.position);
    for (int i = 0 ; i < to_Kernel.length() ; i++ ){
        to_Kernel[i] /= (float)stat.area;
    }

    float weight_wi = kernel(q, to_Kernel);
    float pi_ni = weight_wi * stat.pdn;
    VecType pi = weight_wi * stat.position;
    VecType ni = weight_wi * stat.normal;
    float area = weight_wi * stat.area;
    float norm = weight_wi * stat.norm; 

    float num = pi_ni - glm::dot(pi, ni)/area;
    float denom = norm - (glm::pow(glm::l2Norm(pi),2) / area);
    float u4 = (num/denom)/2;

    VecType num_vec = ni - 2*u4*(pi);
    VecType u123 = num_vec/area;

    num = glm::dot(pi, u123) + u4 * norm;
    float u0 = - num / area;

    VecType center = get_center(u0, u123, u4);
    float radius = get_radius(u0, u123, u4, center);
    return std::pair<VecType, float>(center, radius);
}

/**
 * @brief This function takes the geometric parameters of the sphere and a point to project on it, and returns the new projected point.
 */
template<typename VecType>
VecType project_point (std::pair<VecType, float> sphereInfos, VecType q){
    VecType projectedPoint;

    for (int i = 0; i < q.length(); i++)
        projectedPoint[i] =  q[i] - sphereInfos.first[i];
    
    float factor = (glm::l2Norm(projectedPoint));
    projectedPoint = (projectedPoint / factor);

    float distance_factor = sphereInfos.second;

    projectedPoint = (projectedPoint * distance_factor) + sphereInfos.first;

    return projectedPoint;
}