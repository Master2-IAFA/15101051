#include "ImguiInputOctreeDebug.hpp"

void ImguiInputOctreeDebug::draw(){

    if( ImGui::CollapsingHeader("Octree parameters") ){

        if( ImGui::SliderInt( "Depth", &m_octreeDepth, 0, m_octreeMaxDepth - 1 ) ) drawOctreeAtDepth();

        ImGui::SliderInt( "Max depth", &m_maxDepth, 1, 10 );
        ImGui::SliderInt( "Max Point", &m_maxPoints, 1, 100);

        if( ImGui::Button("Fit") ) fitOctree();
        ImGui::SameLine();
        ImGui::Text(  (std::to_string( m_fitTime ) + " Ms").c_str() );

    }
}

void ImguiInputOctreeDebug::drawOctreeAtDepth(){
    for( int i = 0; i < m_octreeMaxDepth; i++ ){
        m_vectorOctree[ i ]->setEnabled( false );
    }
    m_vectorOctree[ m_octreeDepth ]->setEnabled( true );
}

void ImguiInputOctreeDebug::initVectorOctree(){
    for( auto o : m_vectorOctree ){
        o->remove();
    }
    m_vectorOctree.clear();
    int maxDepth = m_inputOctree->getMaxDepth();
    for( int i = 0; i < maxDepth; i++ ){
        auto d = m_inputOctree->getAtDepth( i );
        auto di = std::vector< BaseOctree3D< InputOctree3D >* >( d.begin(), d.end() );
        m_vectorOctree.push_back( drawOctree( std::to_string( i ), di ) );
        m_vectorOctree[ i ]->setEnabled( false );
    }
}

void ImguiInputOctreeDebug::fitOctree(){
    auto start = std::chrono::high_resolution_clock::now();
    m_inputOctree->fit( m_maxDepth, m_maxPoints );
    initVectorOctree();
    m_octreeMaxDepth = m_inputOctree->getMaxDepth();
    auto stop = std::chrono::high_resolution_clock::now();
    m_fitTime = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
}