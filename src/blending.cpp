#include "blending.hpp"


/**
 * @brief Kernel used into the projection process it takes 2 points and return a scalar
 * @param p : A point that will be compared with q
 * @param q : A point that will be compared with p
 */
float gaussian_mixture (glm::vec3& p, glm::vec3& q){
    // I don't understand all of we need to do.
    return 0.0f;
}

/**
 * @brief Kernel used into the projection process it takes 2 points and return a scalar
 * @param p : A point that will be compared with q
 * @param q : A point that will be compared with p
 */
float rational_kernel (glm::vec3& p, glm::vec3& q){
    // Interpolation : Epsilon = 0.0f
    // approximatin surfaces : Epsilon > 0.0f
    float epsilon = 0.5f;
    float dist = glm::pow(glm::distance(p, q),2);

    // Try to find a good k
    int k = 1;

    float res = glm::pow((dist + epsilon), (-k/2));
    return res;
}

/**
 * @brief This function returns the middle of two points.
 *
 *
 *
 */
glm::vec3 midpoint(const glm::vec3& a, const glm::vec3& b) {
    glm::vec3 ret;
    ret.x = (a.x + b.x) / 2;
    ret.y = (a.y + b.y) / 2;
    ret.z = (a.z + b.z) / 2;
    return ret;
}

/**
 * @brief This function takes a node of an octree and a point and returns a boolean 
 * to say if this point is in the protection sphere
 *
 *
 *
 */
bool is_InProtectionSphere (InputOctree* node, glm::vec3& q) {
    glm::vec3 min = node->getMin();
    glm::vec3 max = node->getMax();
    
    float radius_protectionSphere = (glm::distance(min, max) / 2) * LAMBDA;
    glm::vec3 mid = midpoint(min, max);

    if (glm::distance(mid, q) > radius_protectionSphere)
        return false;
    return true;
}


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
 * @brief Sub-methode of the projection that will recursivly go throught the octree to blend the stats of the algebraic sphere
 * @param octree : Root octree of the input point cloud
 * @param q : Point q that we want to project into the octree
 */
statistics cumul_stats(InputOctree* node, float (*kernel)(glm::vec3&,glm::vec3&), glm::vec3& q){

    statistics father_stats = node->getData();
    double sigma_n = father_stats.area;
    double sigma_nu;

    // If node is a leaf :
    if (!node->hasChildren()){
        // Accumulate statistics over points in the leaf.
}

    auto children = node->getChildren();
    
    for (int i = 0; i < 8; i++){
        if (is_InProtectionSphere(children[i], q))
            cumul_stats(children[i], kernel, q);
        else {
            // As we can see with the equation (5) in the paper, we could estimate it by taking the average position.
            float sigma_n_toFloat = round(sigma_n * pow(10, 7)) / pow(10, 7);
            glm::vec3 averagePosition = father_stats.position / (sigma_n_toFloat);
            float weight = kernel (q, averagePosition);
            return weighted_statistics(father_stats, weight);  
        }
    }

    return father_stats;
}

/**
 * @brief Implementation of the projection methode from the MLoD paper.
 * @param octree : Root octree of the input point cloud
 * @param q : Point q that we want to project into the octree
 */
point projection (InputOctree* octree, float (*kernel)(glm::vec3&,glm::vec3&) ,glm::vec3& q){
    statistics stats = cumul_stats (octree, kernel, q);
    point new_q = {glm::vec3(0.0f), glm::vec3(0.0f)};
    return new_q;
}