#include "ImguiDebug.hpp"

void ImguiDebug::draw(){
    drawInputOctreeDebug();
}

void ImguiDebug::drawInputOctreeDebug(){
    if( ImGui::SliderInt( "Depth", &m_octreeDepth, 0, m_octreeMaxDepth) ) drawOctreeAtDepth();
}

void ImguiDebug::drawOctreeAtDepth(){
    for( int i = 0; i < m_octreeMaxDepth + 1; i++ ){
        m_vectorOctree[ i ]->setEnabled( false );
    }
    m_vectorOctree[ m_octreeDepth ]->setEnabled( true );
}

void ImguiDebug::initVectorOctree(){
    m_vectorOctree.clear();
    int maxDepth = m_inputOctree->getMaxDepth();
    for( int i = 0; i < maxDepth; i++ ){
        auto d = m_inputOctree->getAtDepth( i );
        auto di = std::vector< BaseOctree3D< InputOctree3D >* >( d.begin(), d.end() );
        m_vectorOctree.push_back( drawOctree( std::to_string( i ), di ) );
        m_vectorOctree[ i ]->setEnabled( false );
    }
}