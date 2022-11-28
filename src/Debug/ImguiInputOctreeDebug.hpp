#pragma once

#include <chrono>
#include <memory>
#include <iostream>

#include "polyscope/messages.h"
#include "polyscope/point_cloud.h"
#include "polyscope/pick.h"
#include "polyscope/curve_network.h"
#include "polyscope/polyscope.h"

#include "../Octree/InputOctree.t.hpp"
#include "../AlgebraicSphere.t.hpp"
#include "../PointSet.t.hpp"

#include "../Define.hpp"
#include "../debug.t.hpp"

class ImguiInputOctreeDebug{
    public:
        ImguiInputOctreeDebug( std::shared_ptr< InputOctree3D > inputOctree ): m_inputOctree( inputOctree ), m_octreeMaxDepth( inputOctree->getMaxDepth() ){
            initVectorOctree();
        }

        void draw();
    
    private:

        void initVectorOctree();
        void drawOctreeAtDepth();
        void fitOctree();

        std::vector< polyscope::CurveNetwork * > m_vectorOctree;
        int m_octreeDepth{ 0 };
        int m_octreeMaxDepth{ 0 };
        int m_maxDepth{ 0 };
        int m_maxPoints{ 0 };
        float m_fitTime{ 0.0 };
        std::shared_ptr< InputOctree3D > m_inputOctree;
};