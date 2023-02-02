#pragma once

#include "ImguiFittingDebug.hpp"

void ImguiFittingDebug::draw(){

    ImGui::SliderInt("nb points", &m_numberOfPoints, 10, 50000 );
    ImGui::SameLine();
    if( ImGui::Button( "sample random points" ) ) samplePoints( m_numberOfPoints );

    if( ImGui::RadioButton( "Gaussian_Kernel", m_gaussianKernel ) ){
        m_gaussianKernel = true;
        m_rationnalKernel = false;
        m_kernel = [this]( glm::vec3 a, glm::vec3 b ){ return gaussian_mixture( a, b, m_gaussianK, m_gaussianA, m_gaussianSigma); };
    }
    ImGui::SameLine();
    if( ImGui::RadioButton( "Rationnal Kernel", !m_gaussianKernel ) ){
        m_gaussianKernel = false;
        m_rationnalKernel = true;
        m_kernel = [this]( glm::vec3 a, glm::vec3 b ){ return rational_kernel( a, b, m_rationnalK, m_rationnalEpsilon ); };
    }

    if( m_gaussianKernel ){
        ImGui::SliderInt( "k", &m_gaussianK, 1, 100 );
        ImGui::SliderFloat( "a", &m_gaussianA, 1.0001, 100.0 );
        ImGui::SliderFloat ("sigma", &m_gaussianSigma, 0.0, 100.0);
    }else{
        ImGui::SliderFloat( "k", &m_rationnalK, 0.0, 100.0 );
        ImGui::SliderFloat( "Epsilon", &m_rationnalEpsilon, 0.0, 100.0 );
    }

    if( ImGui::Button( "fit" ) ) fit();

    if( m_fitted ) drawFit();

    if ( ImGui::Button (" Clear Random pc ") ) {
        m_fitted = false;
        polyscope::removePointCloud( m_pointCloud_name );
    }

    fit_One_Point();
}

void ImguiFittingDebug::drawFit(){
    if( ImGui::SliderFloat( "slide ", &m_sliderStatut, 0.0f, 1000.0f) ) slidePoints();
}

void ImguiFittingDebug::slidePoints(){
    float k = m_sliderStatut / 1000.0f;
    std::cout << k << std::endl;
    std::vector< glm::vec3 > p;
    #pragma omp parallel for num_threads( 12 )
    for( int i = 0; i < m_middlePosition.size(); i++ ){
        glm::vec3 start = m_startPosition[ i ];
        glm::vec3 end = m_endPosition[ i ];
        auto direction =  glm::normalize( end - start );
        auto length = glm::length( end - start );
        m_middlePosition[ i ] = m_startPosition[ i ] + k * ( end - start );
    }
    m_pointCloud = polyscope::registerPointCloud( m_pointCloud_name, m_middlePosition );
}

void ImguiFittingDebug::fit(){
    for( int i = 0; i < m_startPosition.size(); i++ ){
        AlgebraicSphere3D sphere;
        point3d point;
        point.pos = glm::vec3( m_startPosition[ i ] );
        point.norm = glm::vec3( 0, 0, 0 );
        statistics3d stat = m_inputOctree->getBlendedStat( point,  [this]( glm::vec3 a, glm::vec3 b ){ return m_kernel( a, b );} );
        //display_statistics( stat );
        sphere.fitSphere( stat, point.pos, [this]( glm::vec3 a, glm::vec3 b ){ return m_kernel( a, b ); });
        m_endPosition[ i ] = sphere.project( point.pos );
        std::cout << "Position of that point ? : " << m_endPosition[ i ][0] << " , " << m_endPosition[ i ][1] << " , " << m_endPosition[ i ][2] << std::endl;
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
    m_pointCloud = polyscope::registerPointCloud( m_pointCloud_name, m_middlePosition );
}

/**
 * @author Léo
 * 
 * @brief This function allows the user to test the fitting process with only one point. It shows you the point, the projected point and the algebraic sphere. 
 * 
 */
void ImguiFittingDebug::fit_One_Point() {
    auto point_name = "fitted_point";
    auto name = "algebraic_sphere_for_point";
    auto name_traversed_octree = "traversed_octree";

    ImGui::Text("Tests with only one point.");

    if (ImGui::Button( "Randomize and fit" )) {
        m_single_fitted = false;
        auto randX = randomFloat( m_inputOctree->getMin().x, m_inputOctree->getMax().x );
        auto randY = randomFloat( m_inputOctree->getMin().y, m_inputOctree->getMax().y );
        auto randZ = randomFloat( m_inputOctree->getMin().z, m_inputOctree->getMax().z );
        m_single_point = glm::vec3 (randX, randY, randZ);

        point3d point;
        point.pos = m_single_point;
        point.norm = glm::vec3( 0, 0, 0 );
        statistics3d stat = m_inputOctree->getBlendedStat( point,  [this]( glm::vec3 a, glm::vec3 b ){ return m_kernel( a, b );} );
        //display_statistics( stat );
        m_sphere_single.fitSphere( stat, point.pos, [this]( glm::vec3 a, glm::vec3 b ){ return m_kernel( a, b ); });
        m_single_point_fitted = m_sphere_single.project( point.pos );
        m_single_fitted = true;
    }
    
    if (m_single_fitted){
        
        point_and_stats_to_sphere (point_name, name, m_single_point_flying, m_single_point_fitted, m_sphere_single);
        draw_traversed_octree (m_inputOctree, m_single_point, name_traversed_octree);

        if( ImGui::SliderFloat( "slide ", &m_sliderStatut_single, 0.0f, 1000.0f) ) slideSinglePoint();

        if (ImGui::Button("Delete single fit")) {
            polyscope::removePointCloud(point_name);
            polyscope::removePointCloud(name);
            polyscope::removeCurveNetwork(name_traversed_octree);
            m_single_fitted = false;
        }
    }

}

void ImguiFittingDebug::slideSinglePoint(){
    float k = m_sliderStatut_single / 1000.0f;
    std::vector< glm::vec3 > p;
    glm::vec3 start = m_single_point;
    glm::vec3 end = m_single_point_fitted;
    auto direction =  glm::normalize( end - start );
    auto length = glm::length( end - start );
    m_single_point_flying = m_single_point + k * ( end - start );
}