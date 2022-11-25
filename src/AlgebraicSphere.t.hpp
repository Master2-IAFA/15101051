#include "AlgebraicSphere.hpp"

template< class VecType, class StatType >
AlgebraicSphere< VecType, StatType >::AlgebraicSphere(){}

template< class VecType, class StatType >
AlgebraicSphere< VecType, StatType >::~AlgebraicSphere(){}

template< class VecType, class StatType >
void AlgebraicSphere< VecType, StatType >::fitSphere( StatType stat, VecType point, float (*kernel)(VecType&,VecType&) ){

    VecType meanPosition = VecType( stat.position );
    meanPosition /= VecType( static_cast<float>(stat.area) );

    float weight_wi = kernel( point, meanPosition );
    VecType pi = weight_wi * stat.position;
    VecType ni = weight_wi * stat.normal;
    float area = weight_wi * stat.area;
    float pi_ni = weight_wi * stat.pdn;
    float norm = weight_wi * stat.norm; 

    float num = pi_ni - glm::dot(pi, ni)/area;
    float denom = norm - ( glm::dot(pi, pi) / area );
    m_u4 = ( num / denom ) / 2;

    VecType num_vec = ni - VecType( 2.0 ) * VecType( m_u4 ) * pi ;
    m_u123 = num_vec / area;

    num = glm::dot( pi, m_u123 ) + m_u4 * norm;
    m_u0 = - num / area;

    this->computeCenter();
    this->computeRadius();
}

template< class VecType, class StatType >
VecType AlgebraicSphere< VecType, StatType >::project( VecType point ){

    VecType projectedPoint;

    for (int i = 0; i < point.length(); i++)
        projectedPoint[i] =  point[i] - m_center[i];
    
    float factor = (glm::l2Norm(projectedPoint));
    projectedPoint = (projectedPoint / factor);

    float distance_factor = m_radius;

    projectedPoint = (projectedPoint * distance_factor) + m_center;

    return projectedPoint;

}

template< class VecType, class StatType >
void AlgebraicSphere< VecType, StatType >::computeCenter(){
    float b = 1.0f / m_u4;
    m_center = -0.5f * m_u123 * b;
}

template< class VecType, class StatType >
void AlgebraicSphere< VecType, StatType >::computeRadius(){
    float b = m_u0 / m_u4;
    auto cTc = glm::dot( m_center, m_center );
    double r = sqrt( cTc - b );
    m_radius = max( 0, r );
}
