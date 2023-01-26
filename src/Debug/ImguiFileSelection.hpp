#pragma once

#include <chrono>
#include <memory>
#include <iostream>
#include <stdlib.h> 
#include <omp.h>
#include <functional>

#include "polyscope/messages.h"
#include "polyscope/point_cloud.h"
#include "polyscope/pick.h"
#include "polyscope/curve_network.h"
#include "polyscope/polyscope.h"

#include "../Octree/InputOctree.t.hpp"
#include "../AlgebraicSphere.t.hpp"
#include "../PointSet.t.hpp"

#include "../kernels.t.hpp"

#include "../Define.hpp"
#include "../debug.t.hpp"

namespace fs = std::filesystem;

template< class VecType, class StatType, class PointType >
class ImguiFileSelection{
    public:

        ImguiFileSelection( PointSet<PointType> *pointSet, std::shared_ptr< InputOctree< VecType, StatType, PointType > > inputOctree, std::string pathToDirectory ): 
            m_pointSet( pointSet ),
            m_pathToDirectory( pathToDirectory ),
            m_inputOctree( inputOctree )
            { init(); }
        
        void draw();
    
    private:

        void loadFile();
        void init();

        std::string m_pathToDirectory{ "../assets/" };
        std::vector<string> m_fileList;
        int m_currentIndex{ 0 };
        PointSet< PointType >* m_pointSet;
        std::shared_ptr< InputOctree< VecType, StatType, PointType > > m_inputOctree;
};