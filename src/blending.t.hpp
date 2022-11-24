#pragma once
#include "blending.hpp"


/**
 * @brief Kernel used into the projection process it takes 2 points and return a scalar
 * @param p : A point that will be compared with q
 * @param q : A point that will be compared with p
 */
template<typename VecType>
float gaussian_mixture (VecType& p, VecType& q){
    float out = 0;
    float base_sigma = 1;

    // This parameters could be changed.
    // WE MUST TRY DIFFERENTS PARAMETERS.
    // This parameter k makes reference to the sum of i => k >= 1.
    int k = 1;
    float a = 1;

    // I think that if K is tiny (like 1 or average) the response is an interpolation.

    // Here, it tries to implement the effective strategy to set appropriate parameters
    // With this king of thing : sigma_i = a^i * sigma with a > 1;
    for (int i = 0; i < k; i++){
        float sigma = pow(a, i) * base_sigma;
        float num = -glm::pow(glm::l2Norm(q-p),2);
        float denom = 2 * pow(sigma,2);
        float fact = (pow(sigma,-3) * exp(num/denom));
        out += fact;
    }
    return out;
}

/**
 * @brief Kernel used into the projection process it takes 2 points and return a scalar
 * @param p : A point that will be compared with q
 * @param q : A point that will be compared with p
 *
 */
template <typename VecType>
float rational_kernel (VecType& p, VecType& q){
    // Interpolation : Epsilon = 0.0f
    // approximation surfaces : Epsilon > 0.0f
    float epsilon = 0.5f;
    float dist = glm::pow(glm::distance(p, q),2);

    // Try to find a good k
    float k = 1.5;

    float res = glm::pow((dist + epsilon), (-k/2));
    return res;
}

/**
 * @brief This function returns the middle of two points.
 *
 * @todo Rendre fonctionnelle l'utilisation de points 2D
 *       Simplement en checkant la taille du glm::vec p ou q et en ajoutant ou non la ret.z
 */
template <typename VecType>
VecType midpoint(VecType& a, VecType& b) {
    VecType ret;
    ret[0] = (a[0] +b[0]) / 2;
    ret[1] += (a[1] + b[1]) / 2;
    if (a.length() > 2){ ret[2] = ( a[2] + b[2] ) / 2;}
    return ret;
}

template <typename statistics, typename VecType>
float signedDistanceToSphere (Octree <statistics, VecType> *node, VecType& q){
    VecType min = node->getMin();
    VecType max = node->getMax();

    float radius_protectionSphere = (glm::distance(min, max) / 2) * LAMBDA;
    VecType mid = midpoint<VecType>(min, max);
    return glm::distance(mid, q) - radius_protectionSphere;

}

/**
 * @brief This function takes a node of an octree and a point and returns a boolean
 * to say if this point is in the protection sphere
 *
 *
 *
 */
template<typename statistics, typename VecType>
bool is_InProtectionSphere (Octree<statistics, VecType> *node, VecType& q){
    return (! (signedDistanceToSphere (node, q) > 0));
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


/**
 *
 * @brief This function takes the father's node and one of its children and returns
 * the gamma function (Equation (7) in the MLoD's paper).
 * It calculates the distance between q and the protection sphere of each node (father and child).
 *
 * @param node : This is the current node that we treat.
 * @param child : This is the child node that we treat too.
 * @param q : This is the point that we want to project.
 *
 */
template<typename statistics, typename VecType>
float gamma_maj (Octree<statistics, VecType> *node, Octree<statistics, VecType> *child, VecType q){
    float distance_to_node = signedDistanceToSphere(node, q);
    float distance_to_child = signedDistanceToSphere(child, q);

    if (distance_to_node <= 0 && distance_to_child <= 0) {
        return 0;
    }
    if (distance_to_node >= 0 && distance_to_child >= 0) {
        return 1;
    }
    float u = (distance_to_child / (distance_to_child - distance_to_node));
    return exp(-exp(1/(u-1)) / pow(u,2));
}

/**
 * @brief Sub-methode of the projection that will recursivly go throught the octree to blend the stats of the algebraic sphere
 * @param octree : Root octree of the input point cloud
 * @param q : Point q that we want to project into the octree
 *
 * quad = dim 2 => 2 ** 2 = 4 noeuds fils.
 * octree = dim 3 => 2 ** 3 = 8 noeuds fils.
 */
template<typename statistics, typename VecType>
statistics cumul_stats(Octree<statistics, VecType> *node, float (*kernel)(VecType&,VecType&), VecType& q){

    statistics father_stats = node->getData();
    double sigma_n = father_stats.area;
    double sigma_nu;

    // If node is a leaf :
    if ( ! node->hasChildren() ){
        float weight = 0.0f;
        // Accumulate statistics over points in the leaf.
        for (VecType p : node->getPoints()){
            weight += kernel (q, p);
        }
        return weighted_statistics(father_stats, weight);
    }
    // float sigma_n_toFloat = round(sigma_n * pow(10, 7)) / pow(10, 7);
    VecType averagePosition = father_stats.position / ((float)sigma_n);
    float weight_averagePos = kernel (q, averagePosition);

    // Case q isn't in the node
    if (! is_InProtectionSphere(node, q)){
        statistics far_away = weighted_statistics(father_stats, weight_averagePos);
        return weighted_statistics(father_stats, weight_averagePos);
    }
    // Let's blend statistics between n and its children
    statistics node_stats;
    std::vector<Octree<statistics, VecType> *> children = node->getChildren();

    for (auto child : children){
        double weight_area = (child->getData().area)/sigma_n;
        float gamma = gamma_maj(node, child, q);
        float weight = weight_averagePos * weight_area * gamma;
        statistics child_stats = cumul_stats(child, kernel, q);
        node_stats = sum_statistics(node_stats, weighted_statistics(child_stats, (1-gamma)));
        node_stats = sum_statistics(node_stats, weighted_statistics(child_stats, weight));
    }
    return node_stats;
}

/**
 * @brief Implementation of the projection methode from the MLoD paper.
 * @param octree : Root octree of the input point cloud
 * @param q : Point q that we want to project into the octree
 */
//  point projection (Octree<statistics, VecType>* octree, float (*kernel)(VecType& ,VecType& ) ,VecType& q);
template<typename statistics, typename point, typename VecType>
std::pair<VecType, float> projection (Octree<statistics, VecType> *octree, float (*kernel)(VecType&,VecType&) ,VecType& q){
    statistics stats = cumul_stats (octree, kernel, q);
    std::pair<VecType, float> sphere = fit_algebraic_sphere(stats, q, kernel);
    return sphere;
    // point new_q;
    // new_q.pos = VecType(0.0f);
    // new_q.norm = VecType(0.0f);
    // return new_q;
}
