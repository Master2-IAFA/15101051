#pragma once

#include <memory>
#include <iostream>

#include "polyscope/messages.h"
#include "polyscope/point_cloud.h"
#include "polyscope/pick.h"
#include "polyscope/curve_network.h"
#include "polyscope/polyscope.h"

#include "Octree/InputOctree.t.hpp"
#include "AlgebraicSphere.t.hpp"
#include "PointSet.t.hpp"

#include "Define.hpp"
#include "debug.t.hpp"

class ImguiDebug{
    public:
        ImguiDebug( std::shared_ptr< InputOctree3D > inputOctree3D, std::shared_ptr< PointSet3D > pointSet3D ){
            m_inputOctree = inputOctree3D;
            m_pointSet = pointSet3D;
            m_octreeMaxDepth = m_inputOctree->getMaxDepth() - 1;
            initVectorOctree();
        }

        void draw();

        void drawInputOctreeDebug();
        
    
    private:

        //slideBar octree
        void drawOctreeAtDepth();

        void initVectorOctree();
        std::vector< polyscope::CurveNetwork * > m_vectorOctree;
        int m_octreeDepth{ 0 };
        int m_octreeMaxDepth{ 0 };
        
        std::shared_ptr< InputOctree3D > m_inputOctree;
        std::shared_ptr< PointSet3D > m_pointSet;
        
};