#pragma once

#include <iostream>
#include <polyscope/polyscope.h>
#include <cmath>

/**
 * @brief class that represent an octree define by an aabb box.
 *        an octree can contain other octrees (8)
 *        an octree can also contain arbitrary data
 * 
 * @tparam Data: the type of data the octree can take.
 */
template<typename Data, typename VecType, typename OctreeType>
class BaseOctree{

public:
    BaseOctree ( OctreeType *_father, int _depth, VecType _min, VecType _max );
    BaseOctree ( int _depth, VecType _min, VecType _max );

    virtual ~BaseOctree ();

    /**
    * @brief split the octree into 8 childrens, and link it as their father.
    */
    virtual void subDivide ();

    /**
    * @brief get all the octrees at the given depth
    */
    std::vector<OctreeType*> getAtDepth ( int depth );

    /**
    * @brief check if the given point is inside the octree using it's min/max points
    */
    bool isPointIn ( VecType p );

    /**
    * @brief return if the octree has children
    *
    * @return true
    * @return false
    */
    bool hasChildren () { return !(m_children[0] == nullptr); }

    /***** setters ******/
    void setFather ( OctreeType* _father ) { m_father = _father; }
    inline void setData ( Data& _data ) { m_data = _data; }
    inline void setPoints (std::vector<VecType> _points) { m_points = _points; }

    /***** getters ******/
    inline const Data& getData () const { return m_data; }
    inline const VecType& getMin () const { return m_min; }
    inline const VecType& getMax () const { return m_max; }
    inline const int getDepth () const { return m_depth; }
    inline std::vector<OctreeType*>& getChildren () { return m_children; }
    inline const int getDim () const { return m_dim; }
    inline std::vector<VecType> getPoints () { return m_points; }

private:
    std::vector<OctreeType*> pGetAtDepth(int depth, std::vector<OctreeType*> vector);
    void init();

protected:
    /**
     * nullptr for root
     */
    OctreeType* m_father { nullptr };
    std::vector<OctreeType*> m_children;
    int m_depth { 0 };
    VecType m_min ;
    VecType m_max ;
    Data m_data;
    std::vector<VecType> m_points;

    /**
     * ambiant space dimension (2 or 3)
     */
    int m_dim;
};