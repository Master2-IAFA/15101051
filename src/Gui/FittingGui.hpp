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

template< class VecType, class StatType, class PointType >
class FittingGui{
    public:
        FittingGui( std::shared_ptr< InputOctree<VecType, StatType, PointType> > inputOctree ): m_inputOctree( inputOctree ){}
        void draw();
        
    private: 
        void drawFit();
        void slidePoints();
        float randomFloat(float a, float b);
        void samplePoints( int n );

        void swapPositions();

        void fit();
        std::string m_fitTime{ "" };

        /** @author Lou */
        void draw_protection_sphere(VecType min, VecType max, float lambda );

        /** @author Léo */
        void unDraw_protection_sphere();
        
        /** This function takes a point and its fitted algebraic 
         * sphere to display the point, the sphere and its projection. 
         * @author Léo 
         * @param [in] name name of the displayed sphere.
         * @param [in] point point that we want to fit a sphere.
         * @param [in] sphere fitted sphere.
         */
        void point_and_stats_to_sphere ( std::string point_name, std::string name, VecType point, VecType end, AlgebraicSphere<VecType, StatType> sphere);
        
        /** get node boxes that where traversed for fitting \p q
         * @todo complete description
        */
        void getTraversedNodes (InputOctree<VecType, StatType, PointType> *current_node_octree, VecType& q, std::vector<std::array<int, 2>>* edges, std::vector<VecType>* nodes);

        /** It displays the traversed nodes of the octree.
         * @author Léo 
         * @param [in] name name of the displayed curve.
         * @param [in] q point that we want to use.
         */
        void draw_traversed_octree (std::shared_ptr< InputOctree<VecType, StatType, PointType > > oct, VecType q, std::string name);

        /** displays sphere on polyscope according to its radius and center */
        void display_sphere( std::string name, VecType center, float radius) ;

    private:
        int m_numberOfPoints = 10;
        bool m_protection_sphere_visible = false;
        float m_sliderStatut{ 0.0 };
        bool m_fitted{ false };

        int m_iterNB{ 1 };
        std::string m_iterTime{ "" };

        std::vector<VecType> m_startPosition;
        std::vector<VecType> m_endPosition;
        std::vector<VecType> m_middlePosition;

        std::string m_pointCloud_name = "Random Points";
        polyscope::PointCloud *m_pointCloud;

        std::shared_ptr< InputOctree<VecType, StatType, PointType> > m_inputOctree;

        //gaussian mixture
        bool m_gaussianKernel{ true };
        bool m_rationnalKernel{ false };
        int m_gaussianK{ 1 }; 
        float m_gaussianA{ 1.0 };
        float m_gaussianSigma{ 5.0 };
        float m_rationnalK{ 0.5 }; 
        float m_rationnalEpsilon{ 1.5 };
        float m_protectionSphere{ 1.3 };
        std::function< float( VecType, VecType ) > m_kernel { [this]( VecType a, VecType b ) { 
            return gaussian_mixture<VecType>( a, b, m_gaussianK, m_gaussianA, m_gaussianSigma ); 
        } };

        // Test fitting with only one point
        void fit_One_Point();
        void slideSinglePoint();
        bool m_single_fitted{ false };
        VecType m_single_point { VecType(0.0f) };
        VecType m_single_point_fitted;

        //Point moving between m_single_point(start pos) and m_single_point_fitted(ending pos);
        VecType m_single_point_flying;
        AlgebraicSphere<VecType, StatType> m_sphere_single;

        float m_sliderStatut_single{ 0.0 };

};


