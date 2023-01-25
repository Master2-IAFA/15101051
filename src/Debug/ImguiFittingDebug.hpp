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

        int m_numberOfPoints = 10;
        float m_sliderStatut{ 0.0 };
        bool m_fitted{ false };

        std::vector<glm::vec3> m_startPosition;
        std::vector<glm::vec3> m_endPosition;
        std::vector<glm::vec3> m_middlePosition;

        std::string m_pointCloud_name = "Random Points";
        polyscope::PointCloud *m_pointCloud;

        std::shared_ptr< InputOctree3D > m_inputOctree;

        //gaussian mixture
        bool m_gaussianKernel{ true };
        bool m_rationnalKernel{ false };
        float m_gaussianK{ 1.0 }; 
        float m_gaussianA{ 1.0 };
        float m_rationnalK{ 0.5 }; 
        float m_rationnalEpsilon{ 1.5 };
        std::function< float( glm::vec3, glm::vec3 ) > m_kernel { [this]( glm::vec3 a, glm::vec3 b ){ return gaussian_mixture( a, b, m_gaussianK, m_gaussianA ); } };
        
        ////////////////////////////////////////

        // Test fitting with only one point
        void fit_One_Point();
        void slideSinglePoint();
        bool m_single_fitted{ false };
        glm::vec3 m_single_point { glm::vec3(0.0f) };
        glm::vec3 m_single_point_fitted;

        //Point moving between m_single_point(start pos) and m_single_point_fitted(ending pos);
        glm::vec3 m_single_point_flying;
        AlgebraicSphere3D m_sphere_single;

        float m_sliderStatut_single{ 0.0 };

};