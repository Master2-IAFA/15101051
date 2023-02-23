#pragma once

/**  Kernel used into the projection process it takes 2 points and return a scalar
 * @param p : A point that will be compared with q
 * @param q : A point that will be compared with p
 */
template<typename VecType>
float gaussian_mixture (VecType& p, VecType& q);

/** Kernel used into the projection process it takes 2 points and return a scalar
 * @author Léo 
 * @param p : A point that will be compared with q
 * @param q : A point that will be compared with p
 * @param k : The number of little gaussian curves that we want. It makes reference to the sum of i => k >= 1.
 * @param a : A coefficient, higher than 1, used to create the sigma of the other curves.
 * @param sigma_zero : The first coefficient, (sigma 0 in the MLODs paper) used to calculate the others gaussians.
 */
template<typename VecType>
float gaussian_mixture (VecType& p, VecType& q, float k, float a, float sigma_zero);

/** Kernel used into the projection process it takes 2 points and return a scalar
 * @author Léo 
 * @param p : A point that will be compared with q
 * @param q : A point that will be compared with p
 * @param k : A parameter used in the rational kernel.
 * @param epsilon : The other used parameter : 0.0 -> interpolation and > 0.0 => Approximation.
 */
template<typename VecType>
float rational_kernel (VecType& p, VecType& q, float k, float epsilon);