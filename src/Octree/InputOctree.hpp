#pragma once

#include "BaseOctree.t.hpp"
#include "../PointSet.t.hpp"
#include "../utils.t.hpp"

/**
 * @brief The inputOctree, is an octree that fit to a pointSet and compute statistics in each node
 * 
 * @tparam VecType 
 * @tparam StatType 
 * @tparam PointType 
 */
template< class VecType, class StatType, class PointType >
class InputOctree: public BaseOctree< StatType, VecType, InputOctree< VecType, StatType, PointType > >{

    public:
        InputOctree( PointSet<PointType> *pointSet ): 
            BaseOctree< StatType, VecType, InputOctree< VecType, StatType, PointType > >( 0, pointSet->getBoundingBox().first.pos, pointSet->getBoundingBox().second.pos )
            { m_pointSet = pointSet; }


        InputOctree( InputOctree< VecType, StatType, PointType > *father, int _depth, VecType _min, VecType _max): 
            BaseOctree< StatType, VecType, InputOctree< VecType, StatType, PointType > >( father, _depth, _min, _max ) {}

        InputOctree( int _depth, VecType _min, VecType _max): BaseOctree< StatType, VecType, InputOctree< VecType, StatType, PointType > >( _depth, _min, _max ){}

        /**
         * @brief fit the octree on the given pointSet
         * @todo implement the max_points constraint
         * @warning the max_points constraint is not implemented yet!
         * @param pointSet 
         * @param max_depth the maximum depth of the octree
         * @param max_points the max number of points per leaf
         */
        void fit( int max_depth, int max_points );

        /**
         * @brief Compute the blended stat over the octree given a point
         * 
         * @param point 
         * @param kernel 
         * @return StatType 
         */
        StatType getBlendedStat( PointType point, float (*kernel)(VecType&,VecType&) );

        bool isInProtectionSphere( VecType point );

        float signedDistanceToProtectionSphere( VecType point );

        inline void setProtectionSphere( float protectionSphere ){ m_protectionSphere = protectionSphere; }
        inline float getProtectionSphere(){ return m_protectionSphere; }

    
    private:

        void recursiveFit( int depth, std::vector<PointType> *points );

        float gamma_maj ( InputOctree<VecType, StatType, PointType> *child, VecType q );

        float m_protectionSphere{ 1.3 };
        PointSet<PointType> *m_pointSet;
};