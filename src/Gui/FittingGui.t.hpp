#pragma once

#include "FittingGui.hpp"
#include "../utils.t.hpp"

template< class VecType, class StatType, class PointType >
void FittingGui<VecType, StatType, PointType>::draw(){

    ImGui::SliderInt("nb points", &m_numberOfPoints, 10, 100000 );
    ImGui::SameLine();
    if( ImGui::Button( "sample random points" ) ) samplePoints( m_numberOfPoints );

    if( ImGui::SliderFloat( "lambda", &m_protectionSphere, 1.0, 2.0 ) ){
        m_inputOctree->setProtectionSphere( m_protectionSphere );
    }

    ImGui::SameLine();

    if (!m_protection_sphere_visible){
        if (ImGui::Button ("Show root's sphere")){
            draw_protection_sphere(m_inputOctree->getMin(), m_inputOctree->getMax(), m_protectionSphere );
        }
    }
    else {
        if (ImGui::Button ("Hide root's sphere")){
            unDraw_protection_sphere();
        }
    }

    if( ImGui::RadioButton( "Gaussian_Kernel", m_gaussianKernel ) ){
        m_gaussianKernel = true;
        m_rationnalKernel = false;
        m_kernel = [this]( VecType a, VecType b ){ return gaussian_mixture( a, b, m_gaussianK, m_gaussianA, m_gaussianSigma ); };
    }
    ImGui::SameLine();
    if( ImGui::RadioButton( "Rationnal Kernel", !m_gaussianKernel ) ){
        m_gaussianKernel = false;
        m_rationnalKernel = true;
        m_kernel = [this]( VecType a, VecType b ){ return rational_kernel( a, b, m_rationnalK, m_rationnalEpsilon ); };
    }

    if( m_gaussianKernel ){
        ImGui::SliderInt( "k", &m_gaussianK, 1, 100 );
        ImGui::SliderFloat( "a", &m_gaussianA, 1.0001, 100.0 );
        ImGui::SliderFloat ("sigma", &m_gaussianSigma, 0.0, 100.0);
    }else{
        ImGui::SliderFloat( "k", &m_rationnalK, 0.0, 100.0 );
        ImGui::SliderFloat( "Epsilon", &m_rationnalEpsilon, 0.0, 100.0 );
    }

    if( ImGui::Button( "fit" ) ) fit( m_startPosition, m_endPosition );
    ImGui::SameLine();
    ImGui::Text( m_fitTime.c_str() );

    ImGui::SliderInt( "iteration", &m_iterNB, 1, 20 );
    if( ImGui::Button( "iterative fit" ) ){
        auto start = std::chrono::high_resolution_clock::now();
        fitNTimes( m_iterNB );
        auto stop = std::chrono::high_resolution_clock::now();
        m_iterTime = std::to_string( std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() ) + " Ms - " + std::to_string( m_iterNB ) + " iterations";
    }
    ImGui::SameLine();
    ImGui::Text( m_iterTime.c_str() );

    if( m_fitted ) drawFit();

    if ( ImGui::Button (" Clear Random pc ") ) {
        m_fitted = false;
        polyscope::removePointCloud( m_pointCloud_name );
    }

    fit_One_Point();
}

template< class VecType, class StatType, class PointType >
void FittingGui<VecType, StatType, PointType>::drawFit(){
    if( ImGui::SliderFloat( "slide ", &m_sliderStatut, 0.0f, 1000.0f) ) slidePoints();
}

template< class VecType, class StatType, class PointType >
void FittingGui<VecType, StatType, PointType>::slidePoints(){
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
void FittingGui<VecType, StatType, PointType>::fit( std::vector<VecType> const start, std::vector<VecType> &end ){

    auto startT = std::chrono::high_resolution_clock::now();

    std::vector<VecType > normals( start.size() );

    #pragma omp parallel for default(none) shared(start, end, normals)
    for( int i = 0; i < start.size(); i++ ){
        AlgebraicSphere<VecType, StatType> sphere;
        PointType point;
        point.pos = VecType( start[ i ] );
        point.norm = VecType( 0.0f );
        StatType stat = m_inputOctree->getBlendedStat( point,  [this]( VecType a, VecType b ){ return m_kernel( a, b );} );
        sphere.fitSphere( stat, point.pos, [this]( VecType a, VecType b ){ return m_kernel( a, b ); });
        end[ i ] = sphere.project( point.pos );
        normals[ i ] = sphere.projectNormal( point.pos );
    }

    if( normals[ 0 ].length() == 3 )
        polyscope::getPointCloud( m_pointCloud_name )->addVectorQuantity("normal", normals );
    else
        polyscope::getPointCloud( m_pointCloud_name )->addVectorQuantity2D( "normal", normals );

    m_fitted = true;

    auto stop = std::chrono::high_resolution_clock::now();
    m_fitTime = std::to_string( std::chrono::duration_cast<std::chrono::milliseconds>(stop - startT).count() ) + " Ms";
}

template< class VecType, class StatType, class PointType >
void FittingGui<VecType, StatType, PointType>::fitNTimes( int n ){

    for( int i = 0; i < n / 2; i++ ){
        fit( m_startPosition, m_endPosition );
        fit( m_endPosition, m_startPosition );
    }

    if( n % 2 != 0 ){
        fit( m_startPosition, m_endPosition );
    }else{
        std::swap( m_startPosition, m_endPosition );
    }

}

template< class VecType, class StatType, class PointType >
float FittingGui<VecType, StatType, PointType>::randomFloat( float a, float b ){
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
}


template< class VecType, class StatType, class PointType >
void FittingGui<VecType, StatType, PointType>::samplePoints( int n ) {
    m_fitted = false;
    m_middlePosition.clear();
    m_endPosition.clear();
    m_startPosition.clear();

    m_startPosition.resize( n );
    m_endPosition.resize( n );
    m_middlePosition.resize( n );

    int vecLength = m_inputOctree->getMin().length();

    for( int i = 0; i < n; ++i ) {
        m_startPosition[i] = VecType(0.0f);
        for (int j = 0;j < vecLength;++j) {
            m_startPosition[i][j] = randomFloat( m_inputOctree->getMin()[j], m_inputOctree->getMax()[j] );
            m_middlePosition[i][j] = m_startPosition[i][j];
        }
    }
    m_pointCloud = (vecLength == 3)?
        polyscope::registerPointCloud( m_pointCloud_name, m_middlePosition ):
        polyscope::registerPointCloud2D( m_pointCloud_name, m_middlePosition );
}

template< class VecType, class StatType, class PointType >
void FittingGui<VecType, StatType, PointType>::fit_One_Point() {
    auto point_name = "fitted_point";
    auto name = "algebraic_sphere_for_point";
    auto name_traversed_octree = "traversed_octree";

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
        m_sphere_single.fitSphere( stat, point.pos, [this]( VecType a, VecType b ){ return m_kernel( a, b ); });
        m_single_point_fitted = m_sphere_single.project( point.pos );
        m_single_fitted = true;
    }
    
    if (m_single_fitted) {
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

template< class VecType, class StatType, class PointType >
void FittingGui<VecType, StatType, PointType>::slideSinglePoint(){
    float k = m_sliderStatut_single / 1000.0f;
    std::vector< VecType > p;
    VecType start = m_single_point;
    VecType end = m_single_point_fitted;
    auto direction =  glm::normalize( end - start );
    auto length = glm::length( end - start );
    m_single_point_flying = m_single_point + k * ( end - start );
}

template<class VecType, class StatType, class PointType>
void FittingGui<VecType, StatType, PointType>::swapPositions() {
    std::swap( m_startPosition, m_endPosition );
}

template< class VecType, class StatType, class PointType >
void FittingGui<VecType, StatType, PointType>::draw_protection_sphere(VecType min, VecType max, float lambda ) {
    drawCube("Protection_cube", min, max);
    float radius_protectionSphere = (glm::distance(min, max) / 2.0) * lambda;
    VecType center_protectionSphere = min + ((max - min)/2.0f) ;
    display_sphere( "Protection_sphere", center_protectionSphere, radius_protectionSphere) ;
    polyscope::getPointCloud( "Protection_sphere" )->setTransparency(0.5f);
}

template< class VecType, class StatType, class PointType >
void FittingGui<VecType, StatType, PointType>::unDraw_protection_sphere() {
    polyscope::removeCurveNetwork( "Protection_cube" );
    polyscope::removePointCloud( "Protection_sphere" );
}

template< class VecType, class StatType, class PointType >
void FittingGui<VecType, StatType, PointType>::point_and_stats_to_sphere (std::string point_name, std::string name, VecType point, VecType end, AlgebraicSphere<VecType, StatType> sphere){
    std::vector<VecType> pos;
    std::vector<VecType> pos_sphere;

    pos.push_back(point);
    pos.push_back(end);
    pos_sphere.push_back(sphere.getCenter());

    polyscope::PointCloud *pc_point = (point.length() == 3)?
                polyscope::registerPointCloud( point_name, pos ):
                polyscope::registerPointCloud2D( point_name, pos );

    pc_point->setPointRadius(0.02);
    polyscope::PointCloud *pc_sphere = (point.length() == 3)?
                polyscope::registerPointCloud( name, pos_sphere ):
                polyscope::registerPointCloud2D( name, pos_sphere );
    pc_sphere->setPointRadius(sphere.getRadius(), false);
}

template< class VecType, class StatType, class PointType >
void FittingGui<VecType, StatType, PointType>::getTraversedNodes (InputOctree<VecType, StatType, PointType> *current_node_octree, VecType& q, std::vector<std::array<int, 2>>* edges, std::vector<VecType>* nodes) {
    if (!current_node_octree->hasChildren())
        return;
    auto children = current_node_octree->getChildren();

    auto cube = build_cube_from_minmax( current_node_octree->getMin(), current_node_octree->getMax());
    int vecLength(current_node_octree->getMin().length());

    int nodes_size = nodes->size();

    for(int j = 0; j < pow (2, vecLength); j++)
        nodes->push_back( cube[j] );

    for (int j = 0;j < pow(2, vecLength);++j) {
        for (int k = j + 1;k < pow(2, vecLength);++k) {
            if (bitDiff(k, j) == 1) {
                edges->push_back({j + nodes_size, k + nodes_size});
            }
        }
    }

    for (int i = 0; i < pow(2, vecLength); i++){
        if (children[i]->isInProtectionSphere( q))
            getTraversedNodes(children[i], q, edges, nodes);
    }
}

template< class VecType, class StatType, class PointType >
void FittingGui<VecType, StatType, PointType>::draw_traversed_octree (std::shared_ptr< InputOctree<VecType, StatType, PointType > > oct, VecType q, std::string name){
    std::vector<std::array<int, 2>> edges ;
    std::vector<VecType> nodes;
    getTraversedNodes(oct.get() ,q ,&edges, &nodes);
    if (q.length() == 3) 
        polyscope::registerCurveNetwork(name, nodes, edges);
    else
        polyscope::registerCurveNetwork2D(name, nodes, edges);
}

template< class VecType, class StatType, class PointType >
void FittingGui<VecType, StatType, PointType>::display_sphere( std::string name, VecType center, float radius) {
    std::vector<VecType> sphere_pos;

    sphere_pos.push_back( center );

    polyscope::PointCloud *pointCloud = (center.length() == 3)?
              polyscope::registerPointCloud( name, sphere_pos ):
              polyscope::registerPointCloud2D( name, sphere_pos );
    pointCloud->setPointRadius(radius, false);
}
