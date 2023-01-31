#include "ImguiInputOctreeDebug.hpp"

template< class VecType, class StatType, class PointType >
void ImguiInputOctreeDebug<VecType, StatType, PointType>::draw(){

    if( ImGui::SliderInt( "Depth", &m_octreeDepth, 0, m_octreeMaxDepth - 1 ) ) drawOctreeAtDepth();

    ImGui::SliderInt( "Max depth", &m_maxDepth, 1, 10 );
    ImGui::SliderInt( "Max Point", &m_maxPoints, 1, 100);

    if( ImGui::Button("Fit") ) fitOctree();
    ImGui::SameLine();
    ImGui::Text(  (std::to_string( m_fitTime ) + " Ms").c_str() );

    drawSphereAtDepth();
}

template< class VecType, class StatType, class PointType >
void ImguiInputOctreeDebug<VecType, StatType, PointType>::drawOctreeAtDepth(){
    for( int i = 0; i < m_octreeMaxDepth; i++ ){
        m_vectorOctree[ i ]->setEnabled( false );
    }
    m_vectorOctree[ m_octreeDepth ]->setEnabled( true );
}

template< class VecType, class StatType, class PointType >
void ImguiInputOctreeDebug<VecType, StatType, PointType>::initVectorOctree(){
    for( auto o : m_vectorOctree ){
        o->remove();
    }
    m_vectorOctree.clear();

    
    int maxDepth = m_inputOctree->getMaxDepth();

    for( int i = 0; i < maxDepth; i++ ){
        auto d = m_inputOctree->getAtDepth( i );
        auto di = std::vector< BaseOctree< StatType, VecType, InputOctree<VecType, StatType, PointType> >* >( d.begin(), d.end() );
        m_vectorOctree.push_back( drawOctree( std::to_string( i ), di ) );
        m_vectorOctree[ i ]->setEnabled( false );
    }
}

template< class VecType, class StatType, class PointType >
void ImguiInputOctreeDebug<VecType, StatType, PointType>::fitOctree(){
    auto start = std::chrono::high_resolution_clock::now();
    m_inputOctree->fit( m_maxDepth, m_maxPoints );
    initVectorOctree();
    m_octreeMaxDepth = m_inputOctree->getMaxDepth();
    auto stop = std::chrono::high_resolution_clock::now();
    m_fitTime = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
}

/**
 * @author Léo 
    * 
    * @brief Allow the user to see the algebraic sphere from stats of the given node (giving a depth and an idx of the node)
    */
template< class VecType, class StatType, class PointType >
void ImguiInputOctreeDebug<VecType, StatType, PointType>::drawSphereAtDepth(){

    auto child = m_inputOctree->getAtDepth(m_depth_forSphere);
    int num_child = 0;
    if (m_depth_forSphere > 0)
        num_child = child.size();
    ImGui::Text("Display the algebraic sphere of a given node.");
    ImGui::SliderInt("Depth of the octree", &m_depth_forSphere, 0, m_maxDepth);
    ImGui::SliderInt("Idx of the octree", &m_idx_forSphere, 0, num_child);

    if (ImGui::Button("Show sphere")) {
        node_stats_to_sphere("Debug_the_node", m_inputOctree.get(), m_depth_forSphere, m_idx_forSphere);
    }
}
