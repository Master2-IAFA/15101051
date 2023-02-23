        if (hasPoint) {
#pragma once
#include "utils.hpp"

/**
 * @brief This function updates the given stats with the information of the given point.
 *
 */
template<typename statistics, typename point>
void statisticsAdd(statistics *stat, point p) {
    // sigma
    stat->area += 1;
    // P_beta
    stat->norm += glm::dot( p.pos, p.pos );
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
 * @brief This function computes the summation of two given statistics.
 */
template <typename statistics>
statistics sum_statistics (const statistics& a, const statistics& b){
    statistics sum_stats;
    sum_stats.position = a.position + b.position;
    sum_stats.normal = a.normal + b.normal;
    sum_stats.norm = a.norm + b.norm;
    sum_stats.area = a.area + b.area;
    sum_stats.pdn = a.pdn + b.pdn;
    return sum_stats;
}

/**
 * @brief This function returns the statistics given in input, multiplied by the factor w.
 */
template<typename statistics>
statistics weighted_statistics (statistics stats, float w) {
    statistics w_stats;
    w_stats.position = stats.position * w;
    w_stats.normal = stats.normal * w;
    w_stats.norm = stats.norm * w;
    w_stats.area = stats.area * w;
    w_stats.pdn = stats.pdn * w;
    return w_stats;
}