#pragma once 
#include "AlgebraicSphere.hpp"

template< class VecType, class StatType >
AlgebraicSphere< VecType, StatType >::AlgebraicSphere(){}

template< class VecType, class StatType >
AlgebraicSphere< VecType, StatType >::~AlgebraicSphere(){}

template< class VecType, class StatType >
void AlgebraicSphere< VecType, StatType >::fitSphere( StatType stat, VecType point, std::function< float( VecType&, VecType& ) > kernel ){

    VecType meanPosition = VecType( stat.position );

    if (stat.area != 0){
        meanPosition /= VecType( static_cast<float>(stat.area) );
    }

    float weight_wi = kernel( point, meanPosition );
    std::cout << "weight : " << weight_wi << weight_wi << std::endl;



    VecType pi = weight_wi * stat.position;
    VecType ni = weight_wi * stat.normal;
    float area = weight_wi * stat.area;
    float pi_ni = weight_wi * stat.pdn;
    float norm = weight_wi * stat.norm; 

    float num = pi_ni - glm::dot( pi, ni )/area;
    float denom = norm - ( glm::dot( pi, pi ) / area );
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

    // VecType projectedPoint;
    // projectedPoint =  glm::normalize( point - m_center );
    // projectedPoint = projectedPoint * VecType( m_radius ) + m_center;
    // return projectedPoint;

    if( m_u4 < 0.00000001 && m_u4 > -0.00000001){
        auto dir = glm::normalize( m_u123 );
        std::cout << "Si nan, c'est peut être de la que ça vient PLAN (projection dans algrebraic sphere)" << std::endl;
        return point - VecType( glm::dot( point - ( m_center ), dir ) ) * dir;
    }

    auto p = point - m_center;
    auto l = glm::length( p );
    
    // Case where l is equal to 0, if we don't test it, we'll have a division by 0.
    if (l == 0){
        std::cout << "Si nan, c'est peut être de la que ça vient l = 0 (projection dans algrebraic sphere)" << std::endl;
        return m_center;
    }
    
    auto q = VecType( m_radius / l ) * p;

    std::cout << "=================" << std::endl;
    std::cout << "m_radius : " << m_radius << std::endl;
    std::cout << "l = " << l << std::endl;
    std::cout << "q = " << q[0] << ", " << q[1] << ", " << q[2] << std::endl;

    return m_center + q;
}

template< class VecType, class StatType >
VecType AlgebraicSphere< VecType, StatType >::projectNormal( VecType point ){

    VecType normal;

    if( m_u4 < 0.0000000001 ){
        normal = glm::normalize( m_u123 );
        return normal;
    }

    normal = VecType( 2 * m_u4 ) * project( point ) + m_u123;
    return normal;

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
    m_radius = std::max( 0.0 , r );
}
