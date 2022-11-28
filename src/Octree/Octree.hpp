#pragma once



#include "BaseOctree.t.hpp"

template< class Data, class VecType >
class Octree: public BaseOctree< Data, VecType, Octree<Data,VecType> >{
    public:
        Octree(Octree<Data,VecType> *father, int _depth, VecType _min, VecType _max): BaseOctree< Data, VecType, Octree<Data,VecType> >( father, _depth, _min, _max ) {}
        Octree(int _depth, VecType _min, VecType _max): BaseOctree< Data, VecType, Octree<Data,VecType> >( _depth, _min, _max ) {}

};

