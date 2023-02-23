#pragma once
#include "utils.hpp"
#include "kernels.t.hpp"

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

template<typename statistics, typename point, typename VecType, typename PointType>
AlgebraicSphere<VecType, statistics> projection (InputOctree<VecType, statistics, PointType> *octree, float (*kernel)(VecType&,VecType&) ,VecType& q){
    statistics stats = cumul_stats (octree, kernel, q);
    //std::pair<VecType, float> sphere = fit_algebraic_sphere(stats, q, kernel);
    AlgebraicSphere<VecType, statistics> sphere;
    sphere.fitSphere( stats, q, kernel );
    return sphere;
    // point new_q;
    // new_q.pos = VecType(0.0f);
    // new_q.norm = VecType(0.0f);
    // return new_q;
}

void generate_gaussian () {
    std::ofstream file ("gaussian.ply");

    file << "ply" << std::endl;
    file << "format ascii 1.0" << std::endl;
    file << "element vertex " << 50 * 50 << std::endl;
    file << "property float x" << std::endl;
    file << "property float y" << std::endl;
    file << "property float z" << std::endl;
    file << "property float nx" << std::endl;
    file << "property float ny" << std::endl;
    file << "property float nz" << std::endl;
    file << "end_header" << std::endl;

    for(float i = 0; i < 50; i++) {
        for( float j = 0; j < 50; j++) {
            float x = (i - 25) / 25.0f;
            float y = (j - 25) / 25.0f;
            file << i/50.0f << " " << exp(-(x*x) - (y*y)) << " " << j/50.0f
                 << " " <<  2 * x * exp(-(x*x) - (y*y)) << " " <<   j/50.0f << " " <<  2 * y * exp(-(x*x) - (y*y))
                 << std::endl;
        }
    }
    file.close();
}

void generate2dGaussian (int nbSamples, int direction) {
    std::string filename = "gaussian2d";
    switch(direction) {
        case 0: { filename += "left.ply"; break; }
        case 1: { filename += "down.ply"; break; }
        case 2: { filename += "right.ply"; break; }
        case 3: { filename += "up.ply"; break; }
        default: { return; }
    }

    std::ofstream file (filename);

    file << "ply" << std::endl;
    file << "format ascii 1.0" << std::endl;
    file << "element vertex " << nbSamples << std::endl;
    file << "property float x" << std::endl;
    file << "property float y" << std::endl;
    file << "property float z" << std::endl;
    file << "property float nx" << std::endl;
    file << "property float ny" << std::endl;
    file << "property float nz" << std::endl;
    file << "end_header" << std::endl;

    for (float i = 0; i < nbSamples; ++i) {
        float size = nbSamples;
        float x = (i - size / 2) / (size / 2);
        switch(direction) {
            case 0: {
                file << - exp(-(x*x)) << " " <<  i / nbSamples << " " <<  0.0f << " " 
                    << - i / nbSamples << " " <<  2 * x * exp(-(x*x)) << " "  <<  0.0f
                    << std::endl;
                break;
            }
            case 1: {
                file << i / nbSamples << " " << - exp(-(x*x)) << " " << 0.0f << " " 
                    <<  2 * x * exp(-(x*x)) << " " << - i / nbSamples << " " <<  0.0f
                    << std::endl;
                break;
            }
            case 2: {
                file << exp(-(x*x)) << " " << i / nbSamples << " " <<  0.0f << " " 
                    <<  i / nbSamples << " " << 2 * x * exp(-(x*x)) << " "  <<  0.0f
                    << std::endl;
                break;
            }
            case 3: {
                file << i / nbSamples << " " << exp(-(x*x)) << " " << 0.0f << " " 
                    <<  2 * x * exp(-(x*x)) << " " << i / nbSamples << " " <<  0.0f
                    << std::endl;
                break;
            }
        }
    }

    file.close();
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

inline int bitDiff (unsigned int n, unsigned int m) {
    int count = 0;

    do {
        if (n % 2 != m % 2) {
            ++count;
        }
        n /= 2;
        m /= 2;
    } while (n != 0 || m != 0);

    return count;
}