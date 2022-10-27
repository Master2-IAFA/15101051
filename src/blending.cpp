#include "blending.hpp"


/**
 * @brief Kernel used into the projection process it takes 2 points and return a scalar
 * @param p : A point that will be compared with q
 * @param q : A point that will be compared with p
 */
float gaussian_mixture (glm::vec3& p, glm::vec3& q){
    return 0.0f;
}

/**
 * @brief Kernel used into the projection process it takes 2 points and return a scalar
 * @param p : A point that will be compared with q
 * @param q : A point that will be compared with p
 */
float rational_kernel (glm::vec3& p, glm::vec3& q){
    return 0.0f;
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


/**
 * @brief Sub-methode of the projection that will recursivly go throught the octree to blend the stats of the algebraic sphere
 * @param octree : Root octree of the input point cloud
 * @param q : Point q that we want to project into the octree
 */
statistics cumul_stats(InputOctree* child, float (*kernel)(glm::vec3&,glm::vec3&), glm::vec3& q){
    statistics stats;
    init_statistics(&stats);
    // std::cout << "Je passe dans un sous noeud." << std::endl;

    if (!child->hasChildren())
        return stats;
    
    auto children = child->getChildren();
    
    for (int i = 0; i < 8; i++){
        if (is_InProtectionSphere(children[i], q))
            cumul_stats(children[i], kernel, q);
    }

    return stats;
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