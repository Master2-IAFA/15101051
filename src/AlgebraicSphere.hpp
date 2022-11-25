#pragma once

#include <functional>

template< class VecType, class StatType >
class AlgebraicSphere{

    public:
        AlgebraicSphere();
        ~AlgebraicSphere();

        /**
         * @brief Compute the Algebraic sphere parameters based on the given statistics
        */
        void fitSphere( StatType stat, VecType point, float (*kernel)(VecType&,VecType&) );

        VecType project( VecType point );

        inline VecType getCenter(){ return m_center; }
        inline double getRadius(){ return m_radius; }


    private:

        void computeCenter();
        void computeRadius();

        double m_u0{ 0 };
        VecType m_u123;
        double m_u4{ 0 };

        VecType m_center;
        double m_radius;
};