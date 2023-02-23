#include "OctreeGui.hpp"

#include <chrono>
#include <memory>
#include <iostream>

template< class VecType, class StatType, class PointType >
void OctreeGui<VecType, StatType, PointType>::draw(){

    if( ImGui::SliderInt( "Depth", &m_octreeDepth, 0, m_octreeMaxDepth - 1 ) ) drawOctreeAtDepth();

    ImGui::SliderInt( "Max depth", &m_maxDepth, 1, 10 );
    ImGui::SliderInt( "Max Point", &m_maxPoints, 1, 100);

    if( ImGui::Button("Fit") ) fitOctree();
    ImGui::SameLine();
    ImGui::Text(  (std::to_string( m_fitTime ) + " Ms").c_str() );

    drawSphereAtDepth();
}

template< class VecType, class StatType, class PointType >
void OctreeGui<VecType, StatType, PointType>::drawOctreeAtDepth(){
    for( int i = 0; i < m_octreeMaxDepth; i++ ){
        m_vectorOctree[ i ]->setEnabled( false );
    }
    m_vectorOctree[ m_octreeDepth ]->setEnabled( true );
}

template< class VecType, class StatType, class PointType >
void OctreeGui<VecType, StatType, PointType>::initVectorOctree(){
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
void OctreeGui<VecType, StatType, PointType>::fitOctree(){
    auto start = std::chrono::high_resolution_clock::now();
    m_inputOctree->fit( m_maxDepth, m_maxPoints );
    initVectorOctree();
    m_octreeMaxDepth = m_inputOctree->getMaxDepth();
    auto stop = std::chrono::high_resolution_clock::now();
    m_fitTime = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
}

/** Allow the user to see the algebraic sphere from stats of 
* the given node (giving a depth and an idx of the node)
* @author Léo 
*/
template< class VecType, class StatType, class PointType >
void OctreeGui<VecType, StatType, PointType>::drawSphereAtDepth(){

    auto child = m_inputOctree->getAtDepth(m_depth_forSphere);
    int num_child = 0;
    if (m_depth_forSphere > 0)
        num_child = child.size();
    ImGui::Text("Display the algebraic sphere of a given node.");
    ImGui::SliderInt("Depth of the octree", &m_depth_forSphere, 0, m_maxDepth);
    ImGui::SliderInt("Idx of the octree", &m_idx_forSphere, 0, num_child - 1);

    if (ImGui::Button("Show sphere")) {
        node_stats_to_sphere("Debug_the_node", m_inputOctree.get(), m_depth_forSphere, m_idx_forSphere);
    }
}

template< class VecType, class StatType, class PointType >
void OctreeGui<VecType, StatType, PointType>::node_stats_to_sphere ( std::string name, InputOctree<VecType, StatType, PointType> * oct, int depth, int num_child){  
    auto octree_depth_three = oct->getAtDepth (depth);

    auto notre_node = (depth == 0)?oct:octree_depth_three.at(num_child);
    auto stat = notre_node->getData();

    float weight_wi = 1.0f;

    VecType pi = weight_wi * stat.position;
    VecType ni = weight_wi * stat.normal;
    float area = weight_wi * stat.area;
    float pi_ni = weight_wi * stat.pdn;
    float norm = weight_wi * stat.norm;

    float num = pi_ni - glm::dot( pi, ni )/area;
    float denom = norm - ( glm::dot( pi, pi ) / area );
    float m_u4 = ( num / denom ) / 2;

    VecType num_vec = ni - VecType( 2.0 ) * VecType( m_u4 ) * pi ;
    VecType m_u123 = num_vec / area;

    auto num_2 = glm::dot( pi, m_u123 ) + m_u4 * norm;
    float m_u0 = - num_2 / area;

    float b = 1.0f / m_u4;
    VecType m_center = -0.5f * m_u123 * b;

    b = m_u0 / m_u4;
    auto cTc = glm::dot( m_center, m_center );
    double r = sqrt( cTc - b );
    float m_radius = std::max( 0.0 , r );

    std::vector<VecType> sphere_pos;

    sphere_pos.push_back( m_center );

    drawCube<VecType>("Le cube", notre_node->getMin(), notre_node->getMax());

    polyscope::PointCloud *pointCloud = (m_center.length() == 3)?
                polyscope::registerPointCloud( name, sphere_pos ):
                polyscope::registerPointCloud2D( name, sphere_pos );
    pointCloud->setPointRadius(m_radius, false);
}

template< class VecType, class StatType, class PointType >
polyscope::CurveNetwork* OctreeGui<VecType, StatType, PointType>::drawOctree(std::string name, std::vector<BaseOctree<StatType, VecType, InputOctree< VecType, StatType, PointType >> *> octree) {
    std::vector<std::array<int, 2>> edges ;
    std::vector<VecType> nodes;

    for(int i = 0; i < octree.size(); i++) {
        //get min/max from each octree cell
        auto min = octree[i]->getMin();
        auto max = octree[i]->getMax();

        //add hypercube coordinates from min/max to the octree
        auto cube = build_cube_from_minmax<VecType>( min , max );
        for(int j = 0; j < int(pow(2, min.length())); j++)
            nodes.push_back( cube[j] );

        //create edges for each hypercube
        for (int j = 0;j < int(pow(2, min.length()));++j) {
            for (int k = j + 1;k < int(pow(2, min.length()));++k) {
                if (bitDiff(k, j) == 1) {
                    edges.push_back({j + int(pow(2, min.length())) * i, k + int(pow(2, min.length())) * i});
                }
            }
        }
    }

    return (octree[0]->getMax().length() == 3)?
                polyscope::registerCurveNetwork(name, nodes, edges):
                polyscope::registerCurveNetwork2D(name, nodes, edges);
}