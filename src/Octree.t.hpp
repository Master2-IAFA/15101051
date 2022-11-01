#include "Octree.hpp"

template<class Data, typename VecType>
Octree<Data, VecType>::Octree(int _depth, VecType _min, VecType _max) {
    m_dim = _min.length();
    m_children.reserve( int(pow(2, m_dim)) );
    for(int i = 0; i < int(pow(2, m_dim)); i++){
        m_children[i] = nullptr;
    }
    m_min = _min;
    m_depth = _depth;
    m_max = _max;

    if (_min.length() != m_dim || _max.length() != m_dim) {
        std::cout << "incoherent dimension on Octree creation" << std::endl;
        this->~Octree();
    }
}

template<class Data, typename VecType>
Octree<Data, VecType>::~Octree(){
    m_father = nullptr;
    //delete _data;
    for(int i = 0; i < int(pow( 2, m_dim )); i++){
        delete m_children[i];
    }
}

template<class Data, typename VecType>
void Octree<Data, VecType>::subDivide(){
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
        m_children.emplace_back( new Octree<Data, VecType>( m_depth + 1, min, max ) );
    }

    for ( int  i = 0 ; i < int(pow( 2, m_dim )) ; ++i ) {
        m_children[i]->setFather(this);
    }
}

template<class Data, typename VecType>
bool Octree<Data, VecType>::isPointIn ( VecType p ) {
    for ( int i = 0 ; i < m_dim ; ++i ) {
        if ( (p[i] < m_min[i]) || (p[i] > m_max[i]) ) {
            return false;
        }
    }

    return true;
}

template<class Data, typename VecType>
std::vector<Octree<Data, VecType>*> Octree<Data, VecType>::getAtDepth(int depth){
    std::vector<Octree<Data, VecType>*> octree;
    std::vector<Octree<Data, VecType>*> stack;
    stack.push_back( this );

    while( !stack.empty() ) {
        Octree<Data, VecType>* current = stack.back();
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
