#pragma once
#include "kernels.hpp"

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

template<typename VecType>
float gaussian_mixture (VecType& p, VecType& q, float k, float a){
    float out = 0;
    float base_sigma = 1;

    // This parameters could be changed.
    // WE MUST TRY DIFFERENTS PARAMETERS.
    // This parameter k makes reference to the sum of i => k >= 1.


    // I think that if K is tiny (like 1 or average) the response is an interpolation.

    // Here, it tries to implement the effective strategy to set appropriate parameters
    // With this king of thing : sigma_i = a^i * sigma with a > 1;
    for (int i = 0; i < k; i++){
        float sigma = pow(a, i) * base_sigma;
        float num = -glm::pow(glm::distance(p, q),2);
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
float rational_kernel (VecType& p, VecType& q, float k, float epsilon){
    // Interpolation : Epsilon = 0.0f
    // approximation surfaces : Epsilon > 0.0f

    float dist = glm::pow(glm::distance(p, q),2);
    float res = glm::pow((dist + epsilon), (-k/2));
    // return sqrt( glm::dot( q - p, q - p) );
    return res;
}
