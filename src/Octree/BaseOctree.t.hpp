#pragma once
#include "BaseOctree.hpp"

template<class Data, typename VecType, typename OctreeType>
BaseOctree<Data, VecType, OctreeType>::BaseOctree(OctreeType *_father, int _depth, VecType _min, VecType _max) {
    m_father = _father;
    m_min = _min;
    m_depth = _depth;
    m_max = _max;
    init();
    
}

template<class Data, typename VecType, typename OctreeType>
BaseOctree<Data, VecType, OctreeType>::BaseOctree( int _depth, VecType _min, VecType _max ) {
    m_min = _min;
    m_depth = _depth;
    m_max = _max;
    init(); 
}

template<class Data, typename VecType, typename OctreeType>
BaseOctree<Data, VecType, OctreeType>::~BaseOctree(){
    m_father = nullptr;
    //delete _data;
    for(int i = 0; i < int(pow( 2, m_dim )); i++){
        delete m_children[i];
    }
}

template<class Data, typename VecType, typename OctreeType>
void BaseOctree<Data, VecType, OctreeType>::init(){
    m_dim = m_min.length();
    m_children.reserve( int(pow(2, m_dim)) );
    for(int i = 0; i < int(pow(2, m_dim)); i++){
        m_children[i] = nullptr;
    }

    if (m_min.length() != m_dim || m_max.length() != m_dim) {
        std::cout << "incoherent dimension on Octree creation" << std::endl;
        this->~BaseOctree();
    }
}


template<class Data, typename VecType, typename OctreeType>
void BaseOctree<Data, VecType, OctreeType>::subDivide(){
    VecType min;
    VecType max;
    VecType center;

    for ( int i = 0 ; i < m_dim ; ++i ) {
        center[i] = (m_min[i] + m_max[i]) / 2;
    }

    m_children.clear();
    m_children.reserve( size_t(pow( 2, m_dim )) );

    for ( int i = 0 ; i < int(pow( 2, m_dim )) ; ++i ) {
        for ( int j = 0 ; j < m_dim ; ++j ) {
            if ( (i / int(pow( 2, j ))) % 2 == 0 ) {
                min[j] = m_min[j];
                max[j] = center[j];
            }
            else {
                min[j] = center[j];
                max[j] = m_max[j];
            }
        }
        m_children.emplace_back( new OctreeType( dynamic_cast<OctreeType*>( this ), m_depth + 1, min, max ) );
    }


}

template<class Data, typename VecType, typename OctreeType>
bool BaseOctree<Data, VecType, OctreeType>::isPointIn ( VecType p ) {
    for ( int i = 0 ; i < m_dim ; ++i ) {
        if ( (p[i] < m_min[i]) || (p[i] > m_max[i]) ) {
            return false;
        }
    }

    return true;
}

template<class Data, typename VecType, typename OctreeType>
std::vector<OctreeType*> BaseOctree<Data, VecType, OctreeType>::getAtDepth(int depth){
    std::vector<OctreeType*> octree;
    std::vector<OctreeType*> stack;
    stack.push_back( dynamic_cast<OctreeType*>(this) );

    if( depth == 0 ) return stack;

    if( this->hasChildren() ){
        for( int i = 0; i < int(pow(2, m_dim)); i++ ){
            stack.push_back( m_children[i]  );
        }
    }

    while( !stack.empty() ) {
        OctreeType* current = stack.back();
        stack.pop_back();

        if( current->hasChildren() ) {
            auto children = current->getChildren();
            for(int i = 0; i < int(pow(2, m_dim)); i++){
                if( children[i]->getDepth() == depth ) {
                    octree.push_back( children[i] );
                }
                else {
                    stack.push_back( children[i] );
                }
            }
        }
    }

    return octree;
}

template<class Data, typename VecType, typename OctreeType>
int BaseOctree<Data, VecType, OctreeType>::getMaxDepth(){
    if( !hasChildren() ) return 0;
    int maxi = 0;
    for( auto child : m_children ){
        maxi = std::max( maxi, child->getMaxDepth() );
    }
    return 1 + maxi;
}

