#pragma once

#include <chrono>
#include <memory>
#include <iostream>
#include <stdlib.h> 

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

class ImguiFittingDebug{
    public:

        ImguiFittingDebug( std::shared_ptr< InputOctree3D > inputOctree ): m_inputOctree( inputOctree ){}

        void draw();
        


    private:

        void drawFit();

        void slidePoints();

        float randomFloat(float a, float b);
        void samplePoints( int n );
        void fit();

        int m_numberOfPoints;
        float m_sliderStatut{ 0.0 };
        bool m_fitted{ false };

        std::vector<glm::vec3> m_startPosition;
        std::vector<glm::vec3> m_endPosition;
        std::vector<glm::vec3> m_middlePosition;

        polyscope::PointCloud *m_pointCloud;

        std::shared_ptr< InputOctree3D > m_inputOctree;
        
};