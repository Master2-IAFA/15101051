#pragma once

#include "polyscope/messages.h"
#include "polyscope/point_cloud.h"
#include "polyscope/pick.h"
#include "polyscope/curve_network.h"
#include "polyscope/polyscope.h"

#include "../Octree/InputOctree.t.hpp"
#include "../AlgebraicSphere.t.hpp"
#include "../PointSet.t.hpp"

template< class VecType, class StatType, class PointType >
class OctreeGui{
    public:
        OctreeGui( std::shared_ptr< InputOctree<VecType, StatType, PointType> > inputOctree ): m_inputOctree( inputOctree ), m_octreeMaxDepth( inputOctree->getMaxDepth() ){
            initVectorOctree();
        }

        void draw();
    
    private:
        void initVectorOctree();
        void drawOctreeAtDepth();
        void fitOctree();

        /** This function take an octree, a depth and an idx for the node at the
         *  given depth and show its algebraic sphere. 
         * @author Léo 
         * @param name name of the displayed sphere.
         * @param oct 
         * @param depth Depth of the node we want
         * @param num_child Num of the node for the given depth
         */
        void node_stats_to_sphere ( std::string name, InputOctree<VecType, StatType, PointType> * oct, int depth, int num_child);

        /** Allow the user to see the algebraic sphere from stats of the given node
         *  (giving a depth and an idx of the node)
         * @author Léo
         */
        void drawSphereAtDepth();

        polyscope::CurveNetwork* drawOctree(std::string name, std::vector<BaseOctree<StatType, VecType, InputOctree< VecType, StatType, PointType >> *> octree);

    private:
        std::vector< polyscope::CurveNetwork * > m_vectorOctree;
        int m_octreeDepth{ 0 };
        int m_octreeMaxDepth{ 0 };
        int m_maxDepth{ 0 };
        int m_maxPoints{ 0 };
        float m_fitTime{ 0.0 };
        int m_depth_forSphere{ 0 };
        int m_idx_forSphere{ 0 };
        std::shared_ptr< InputOctree<VecType, StatType, PointType> > m_inputOctree;
};
