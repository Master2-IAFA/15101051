#include "Octree.hpp"

template<class Data>
Octree<Data>::Octree(int _depth, std::vector<float> _min, std::vector<float> _max, int _dim) {
    m_children.reserve( int(pow(2, _dim)) );
    for(int i = 0; i < int(pow(2, _dim)); i++){
        m_children[i] = nullptr;
    }
    m_min = _min;
    m_depth = _depth;
    m_max = _max;
    m_dim = _dim;

    if (_min.size() != _dim || _max.size() != _dim) {
        std::cout << "incoherent dimension on Octree creation" << std::endl;
        this->~Octree();
    }
}

template<class Data>
Octree<Data>::~Octree(){
    m_father = nullptr;
    //delete _data;
    for(int i = 0; i < 8; i++){
        delete m_children[i];
    }
}

template<class Data>
void Octree<Data>::subDivide(){
    std::vector<float> min;
    std::vector<float> max;
    std::vector<float> center;
    for ( int i = 0 ; i < m_dim ; ++i ) {
        center.emplace_back( (m_min[i] + m_max[i]) / 2 );
    }

    if (m_dim == 2) {
        //bottom left
        m_children[0] = new Octree<Data>( m_depth + 1, m_min, center, m_dim );

        //bottom right
        min.emplace_back( (m_min[0] + m_max[0]) / 2 );
        min.emplace_back( m_min[1] );

        max.emplace_back( m_max[0] );
        max.emplace_back( (m_min[1] + m_max[1]) / 2 );

        m_children[1] = new Octree<Data>( m_depth + 1, min, max, m_dim );

        min.clear();
        max.clear();

        //top left
        min.emplace_back( m_min[0] );
        min.emplace_back( (m_min[1] + max[1]) / 2 );

        max.emplace_back( (m_min[0] + m_max[0]) / 2 );
        max.emplace_back( m_max[1] );

        m_children[2] = new Octree<Data>( m_depth + 1, min, max, m_dim );

        min.clear();
        max.clear();

        //top right
        m_children[3] = new Octree<Data>( m_depth + 1, center, m_max, m_dim );
    }

    if (m_dim == 3) {
        //bottom front left
        m_children[0] = new Octree<Data>( m_depth + 1, m_min, center, m_dim );

        //bottom front right
        min.emplace_back( (m_min[0] + m_max[0]) / 2 );
        min.emplace_back( m_min[1] );
        min.emplace_back( m_min[2] );

        max.emplace_back( m_max[0] );
        max.emplace_back( (m_min[1] + m_max[1]) / 2 );
        max.emplace_back( (m_min[2] + m_max[2]) / 2 );

        m_children[1] = new Octree<Data>( m_depth + 1, min, max, m_dim );

        min.clear();
        max.clear();

        //bottom back left
        min.emplace_back( m_min[0] );
        min.emplace_back( (m_min[1] + m_max[1]) / 2 );
        min.emplace_back( m_min[2] );

        max.emplace_back( (m_min[0] + m_max[0]) / 2 );
        max.emplace_back( m_max[1] );
        max.emplace_back( (m_min[2] + m_max[2]) / 2 );

        m_children[2] = new Octree<Data>( m_depth + 1, min, max, m_dim );

        min.clear();
        max.clear();

        //bottom back right
        min.emplace_back( (m_min[0] + m_max[0]) / 2 );
        min.emplace_back( (m_min[1] + m_max[1]) / 2 );
        min.emplace_back( m_min[2] );

        max.emplace_back( m_max[0] );
        max.emplace_back( m_max[1] );
        max.emplace_back( (m_min[2] + m_max[2]) / 2 );

        m_children[3] = new Octree<Data>( m_depth + 1, min, max, m_dim );

        min.clear();
        max.clear();

        //top front left
        min.emplace_back( m_min[0] );
        min.emplace_back( m_min[1] );
        min.emplace_back( (m_min[2] + m_max[2]) / 2 );

        max.emplace_back( (m_min[0] + m_max[0]) / 2 );
        max.emplace_back( (m_min[1] + m_max[1]) / 2 );
        max.emplace_back( m_max[2] );

        m_children[4] = new Octree<Data>( m_depth + 1, min, max, m_dim );

        min.clear();
        max.clear();

        //top front right
        min.emplace_back( (m_min[0] + m_max[0]) / 2 );
        min.emplace_back( m_min[1] );
        min.emplace_back( (m_min[2] + m_max[2]) / 2 );

        max.emplace_back( m_max[0] );
        max.emplace_back( (m_min[1] + m_max[1]) / 2 );
        max.emplace_back( m_max[2] );

        m_children[5] = new Octree<Data>( m_depth + 1, min, max, m_dim );

        min.clear();
        max.clear();

        //top back left
        min.emplace_back( m_min[0] );
        min.emplace_back( (m_min[1] + m_max[1]) / 2 );
        min.emplace_back( (m_min[2] + m_max[2]) / 2 );

        max.emplace_back( (m_min[0] + m_max[0]) / 2 );
        max.emplace_back( m_max[1] );
        max.emplace_back( m_max[2] );

        m_children[6] = new Octree<Data>( m_depth + 1, min, max, m_dim );

        min.clear();
        max.clear();

        //top back right
        m_children[7] = new Octree<Data>(m_depth + 1, center, m_max, m_dim );
    }

    for ( int  i = 0 ; i < int(pow(2, m_dim)) ; ++i ) {
        m_children[i]->setFather(this);
    }
}

template<class Data>
bool Octree<Data>::isPointIn(std::vector<float> p) {
    std::vector<float> min = this->getMin();
    std::vector<float> max = this->getMax();
    //if you want to vizualise cube and point on polyscope
    //std::vector<glm::vec3> cube = build_cube_from_minmax(min, max);
    //cube.push_back(p) ;

    if ((p[0] > min[0]) && (p[0] < max[0]) &&
        (p[1] > min[1]) && (p[1] < max[1]) &&
        (p[2] > min[2]) && (p[2] < max[2])) {
        //std::cout << "point is inside \n" ;
        return true ;
    }
    //std::cout << "point is outside \n" ;
    return false ;
}

template<class Data>
std::vector<Octree<Data>*> Octree<Data>::getAtDepth(int depth){
    std::vector<Octree<Data>*> octree;
    std::vector<Octree<Data>*> stack;
    stack.push_back( this );

    while( !stack.empty() ) {
        Octree<Data>* current = stack.back();
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
