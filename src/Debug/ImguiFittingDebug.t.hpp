#pragma once

#include "ImguiFittingDebug.hpp"

template< class VecType, class StatType, class PointType >
void ImguiFittingDebug<VecType, StatType, PointType>::draw(){

    ImGui::SliderInt("nb points", &m_numberOfPoints, 10, 50000 );
    ImGui::SameLine();
    if( ImGui::Button( "sample random points" ) ) samplePoints( m_numberOfPoints );

    if( ImGui::RadioButton( "Gaussian_Kernel", m_gaussianKernel ) ){
        m_gaussianKernel = true;
        m_rationnalKernel = false;
        m_kernel = [this]( VecType a, VecType b ){ return gaussian_mixture( a, b, m_gaussianK, m_gaussianA ); };
    }
    ImGui::SameLine();
    if( ImGui::RadioButton( "Rationnal Kernel", !m_gaussianKernel ) ){
        m_gaussianKernel = false;
        m_rationnalKernel = true;
        m_kernel = [this]( VecType a, VecType b ){ return rational_kernel( a, b, m_rationnalK, m_rationnalEpsilon ); };
    }

    if( m_gaussianKernel ){
        ImGui::SliderFloat( "K", &m_gaussianK, 0.0, 5.0 );
        ImGui::SameLine();
        ImGui::SliderFloat( "A", &m_gaussianA, 0.0, 5.0 );
    }else{
        ImGui::SliderFloat( "K", &m_rationnalK, 0.0, 5.0 );
        ImGui::SameLine();
        ImGui::SliderFloat( "Epsilon", &m_rationnalEpsilon, 0.0, 5.0 );
    }

    if( ImGui::Button( "fit" ) ) fit();

    if( m_fitted ) drawFit();

    if ( ImGui::Button (" Clear Random pc ") ) {
        m_fitted = false;
        polyscope::removePointCloud( m_pointCloud_name );
    }

    fit_One_Point();
}

template< class VecType, class StatType, class PointType >
void ImguiFittingDebug<VecType, StatType, PointType>::drawFit(){
    if( ImGui::SliderFloat( "slide ", &m_sliderStatut, 0.0f, 1000.0f) ) slidePoints();
}

template< class VecType, class StatType, class PointType >
void ImguiFittingDebug<VecType, StatType, PointType>::slidePoints(){
    float k = m_sliderStatut / 1000.0f;

    std::vector< VecType > p;
    p.resize( m_startPosition.size() );
    #pragma omp parallel for num_threads( 12 )
    for( int i = 0; i < m_middlePosition.size(); i++ ){
        VecType start = m_startPosition[ i ];
        VecType end = m_endPosition[ i ];
        auto direction =  glm::normalize( end - start );
        auto length = glm::length( end - start );
        p[ i ] = m_startPosition[ i ] + k * ( end - start );

    }

    if( p[ 0 ].length() == 3 )
        m_pointCloud->updatePointPositions( p );
    else
        m_pointCloud->updatePointPositions2D( p );

}

template< class VecType, class StatType, class PointType >
void ImguiFittingDebug<VecType, StatType, PointType>::fit(){

    std::vector<VecType > normals;

    for( int i = 0; i < m_startPosition.size(); i++ ){
        AlgebraicSphere<VecType, StatType> sphere;
        PointType point;
        point.pos = VecType( m_startPosition[ i ] );
        point.norm = VecType( 0.0f );
        StatType stat = m_inputOctree->getBlendedStat( point,  [this]( VecType a, VecType b ){ return m_kernel( a, b );} );
        sphere.fitSphere( stat, point.pos, [this]( VecType a, VecType b ){ return m_kernel( a, b ); });
        m_endPosition[ i ] = sphere.project( point.pos );
        normals.push_back( sphere.projectNormal( point.pos ) );
    }

    if( normals[ 0 ].length() == 3 )
        polyscope::getPointCloud( m_pointCloud_name )->addVectorQuantity("normal", normals );
    else
        polyscope::getPointCloud( m_pointCloud_name )->addVectorQuantity2D( "normal", normals );

    m_fitted = true;
}

template< class VecType, class StatType, class PointType >
float ImguiFittingDebug<VecType, StatType, PointType>::randomFloat( float a, float b ){
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
}


template< class VecType, class StatType, class PointType >
void ImguiFittingDebug<VecType, StatType, PointType>::samplePoints( int n ){

    m_fitted = false;
    m_middlePosition.clear();
    m_endPosition.clear();
    m_startPosition.clear();

    m_startPosition.resize( n );
    m_endPosition.resize( n );
    m_middlePosition.resize( n );

    int vecLength = m_inputOctree->getMin().length();

    #pragma omp parallel for
    for( int i = 0; i < n; ++i ) {
        m_startPosition[i] = VecType(0.0f);
        for (int j = 0;j < vecLength;++j) {
            m_startPosition[i][j] = randomFloat( m_inputOctree->getMin()[j], m_inputOctree->getMax()[j] );
            m_middlePosition[i][j] = randomFloat( m_inputOctree->getMin()[j], m_inputOctree->getMax()[j] );
        }
    }
    if (vecLength == 3)
        m_pointCloud = polyscope::registerPointCloud( m_pointCloud_name, m_middlePosition );
    else
        m_pointCloud = polyscope::registerPointCloud2D( m_pointCloud_name, m_middlePosition );
}

/**
 * @author Léo
 * 
 * @brief This function allows the user to test the fitting process with only one point. It shows you the point, the projected point and the algebraic sphere. 
 * 
 */
template< class VecType, class StatType, class PointType >
void ImguiFittingDebug<VecType, StatType, PointType>::fit_One_Point() {
    auto point_name = "fitted_point";
    auto name = "algebraic_sphere_for_point";

    ImGui::Text("Tests with only one point.");

    if (ImGui::Button( "Randomize and fit" )) {
        m_single_fitted = false;
        int vecLength = m_inputOctree->getMin().length();

        m_single_point = VecType( 0.0f );
        for (int j = 0;j < vecLength;++j) {
            m_single_point[j] = randomFloat( m_inputOctree->getMin()[j], m_inputOctree->getMax()[j] );
        }

        PointType point;
        point.pos = m_single_point;
        point.norm = VecType( 0 );
        StatType stat = m_inputOctree->getBlendedStat( point,  [this]( VecType a, VecType b ){ return m_kernel( a, b );} );
        //display_statistics( stat );
        m_sphere_single.fitSphere( stat, point.pos, [this]( VecType a, VecType b ){ return m_kernel( a, b ); });
        m_single_point_fitted = m_sphere_single.project( point.pos );
        m_single_fitted = true;
    }
    
    if (m_single_fitted){
        
        point_and_stats_to_sphere (point_name, name, m_single_point_flying, m_single_point_fitted, m_sphere_single);

        if( ImGui::SliderFloat( "slide ", &m_sliderStatut_single, 0.0f, 1000.0f) ) slideSinglePoint();

        if (ImGui::Button("Delete single fit")) {
            polyscope::removePointCloud(point_name);
            polyscope::removePointCloud(name);
            m_single_fitted = false;
        }
    }

}

template< class VecType, class StatType, class PointType >
void ImguiFittingDebug<VecType, StatType, PointType>::slideSinglePoint(){
    float k = m_sliderStatut_single / 1000.0f;
    std::vector< VecType > p;
    VecType start = m_single_point;
    VecType end = m_single_point_fitted;
    auto direction =  glm::normalize( end - start );
    auto length = glm::length( end - start );
    m_single_point_flying = m_single_point + k * ( end - start );
}
