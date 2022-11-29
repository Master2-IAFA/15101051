#pragma once

#include "ImguiFittingDebug.hpp"

void ImguiFittingDebug::draw(){

    ImGui::SliderInt("nb points", &m_numberOfPoints, 10, 50000 );
    ImGui::SameLine();
    if( ImGui::Button( "sample random points" ) ) samplePoints( m_numberOfPoints );
    if( ImGui::Button( "fit" ) ) fit();

    if( m_fitted ) drawFit();
}

void ImguiFittingDebug::drawFit(){
    if( ImGui::SliderFloat( "slide ", &m_sliderStatut, 0.0f, 1000.0f) ) slidePoints();
}

void ImguiFittingDebug::slidePoints(){
    float k = m_sliderStatut / 1000.0f;
    std::cout << k << std::endl;
    std::vector< glm::vec3 > p;
    for( int i = 0; i < m_middlePosition.size(); i++ ){
        glm::vec3 start = m_startPosition[ i ];
        glm::vec3 end = m_endPosition[ i ];
        auto direction =  glm::normalize( end - start );
        auto length = glm::length( end - start );
        m_middlePosition[ i ] = k>0.5f? m_endPosition[i] : m_startPosition[ i ];
    }
    m_pointCloud = polyscope::registerPointCloud( "Random Points", m_middlePosition );
}

void ImguiFittingDebug::fit(){
    for( int i = 0; i < m_startPosition.size(); i++ ){
        AlgebraicSphere3D sphere;
        point3d point;
        point.pos = glm::vec3( m_startPosition[ i ] );
        point.norm = glm::vec3( 0, 0, 0 );
        statistics3d stat = m_inputOctree->getBlendedStat( point, &gaussian_mixture );
        display_statistics( stat );
        sphere.fitSphere( stat, point.pos, &gaussian_mixture );
        m_endPosition[ i ] = sphere.project( point.pos );
        std::cout << m_endPosition[ i ].x << " " << m_endPosition[ i ].y << " " << m_endPosition[ i ].z << std::endl;
    }

    m_fitted = true;
}

float ImguiFittingDebug::randomFloat( float a, float b ){
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
}


void ImguiFittingDebug::samplePoints( int n ){

    m_fitted = false;
    m_middlePosition.clear();
    m_endPosition.clear();
    m_startPosition.clear();

    m_startPosition.resize( n );
    m_endPosition.resize( n );
    m_middlePosition.resize( n );

    for( int i = 0; i < n; i++ ){
        auto randX = randomFloat( m_inputOctree->getMin().x, m_inputOctree->getMax().x );
        auto randY = randomFloat( m_inputOctree->getMin().y, m_inputOctree->getMax().y );
        auto randZ = randomFloat( m_inputOctree->getMin().z, m_inputOctree->getMax().z );
        m_startPosition[i] = glm::vec3( randX, randY, randZ );
        m_middlePosition[i] = glm::vec3( randX, randY, randZ );
    }
    m_pointCloud = polyscope::registerPointCloud( "Random Points", m_middlePosition );
}