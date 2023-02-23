#pragma once

#include <functional>

#include "BaseOctree.t.hpp"
#include "../PointSet.t.hpp"

/** The inputOctree, is an octree that fits to a pointSet and computes statistics in each node
 * 
 * @tparam VecType glm::vec3 or vec2
 * @tparam StatType octree label type (aligns with VecType)
 * @tparam PointType pointset type (aligns with VecType)
 */
template< class VecType, class StatType, class PointType >
class InputOctree: public BaseOctree< StatType, VecType, InputOctree< VecType, StatType, PointType > >{
    public:
        InputOctree( PointSet<PointType> *pointSet ):
            BaseOctree< StatType, VecType, InputOctree< VecType, StatType, PointType > >( 0, pointSet->getBoundingBox().first.pos, pointSet->getBoundingBox().second.pos ),
            m_pointSet( pointSet ) {
                m_protectionSphere = std::make_shared< float >( 2.0 );
            }

        /** fit the octree on the given pointSet
         * @todo implement the max_points constraint
         * @param [in] max_depth the maximum depth of the octree
         * @param [in] max_points the max number of points per leaf
         */
        void fit( int max_depth, int max_points );

        /**  Compute the blended stat over the octree given a point */
        StatType getBlendedStat( PointType point, std::function< float( VecType&, VecType& ) > kernel);

        bool isInProtectionSphere( VecType point );

        /** (euclidian distance between \p point and node center) - radius x protectionSphereCoeff */
        float signedDistanceToProtectionSphere( VecType point );

        inline void setProtectionSphere( float protectionSphere ){ *m_protectionSphere = protectionSphere; }
        inline float getProtectionSphere(){ return *m_protectionSphere; }
    
    private:

        //private Constructor that are used by the BaseOctree class ( in subdivide function )
        InputOctree( InputOctree< VecType, StatType, PointType > *father, int _depth, VecType _min, VecType _max): 
            BaseOctree< StatType, VecType, InputOctree< VecType, StatType, PointType > >( father, _depth, _min, _max ),
            m_protectionSphere( father->m_protectionSphere ) {}

        InputOctree( int _depth, VecType _min, VecType _max): BaseOctree< StatType, VecType, InputOctree< VecType, StatType, PointType > >( _depth, _min, _max ){}

        void recursiveFit( int depth, std::vector<PointType> *points );
        float gamma_maj ( InputOctree<VecType, StatType, PointType> *child, VecType q );

        std::shared_ptr< float > m_protectionSphere;
        PointSet<PointType> *m_pointSet;
        std::vector<PointType> m_points;
    
    
    //let the BaseOctree class call the private constructor
    friend class BaseOctree< StatType, VecType, InputOctree< VecType, StatType, PointType > >;
};