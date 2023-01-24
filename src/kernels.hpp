#pragma once

/**
 * @brief Kernel used into the projection process it takes 2 points and return a scalar
 * @param p : A point that will be compared with q
 * @param q : A point that will be compared with p
 */
template<typename VecType>
float gaussian_mixture (VecType& p, VecType& q);

template<typename VecType>
float gaussian_mixture (VecType& p, VecType& q, float k, float a);



/**
 * @brief Kernel used into the projection process it takes 2 points and return a scalar
 * @param p : A point that will be compared with q
 * @param q : A point that will be compared with p
 */
template<typename VecType>
float rational_kernel (VecType& p, VecType& q);

template<typename VecType>
float rational_kernel (VecType& p, VecType& q, float k, float epsilon);