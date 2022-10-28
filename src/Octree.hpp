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
template<typename Data, typename VecType>
class Octree{

public:
    Octree (int _depth, VecType _min, VecType _max);

    ~Octree ();

    /**
    * @brief split the octree into 8 childrens, and link it as their father.
    */
    void subDivide ();

    /**
    * @brief get all the octrees at the given depth
    */
    std::vector<Octree<Data, VecType>*> getAtDepth ( int depth );

    /**
    * @brief check if the given point is inside the octree using it's min/max points
    */
    bool isPointIn ( std::vector<float> p );

    /**
    * @brief return if the octree has children
    *
    * @return true
    * @return false
    */
    bool hasChildren () { return !(m_children[0] == nullptr); }

    /***** setters ******/
    inline void setFather ( Octree<Data, VecType>* _father ) { m_father = _father; }
    inline void setData ( Data& _data ) { m_data = _data; }

    /***** getters ******/
    inline const Data& getData () const { return m_data; }
    inline const VecType& getMin () const { return m_min; }
    inline const VecType& getMax () const { return m_max; }
    inline const int getDepth () const { return m_depth; }
    inline std::vector<Octree<Data, VecType>*>& getChildren () { return m_children; }
    inline const int getDim () const { return m_dim; }

private:
    std::vector<Octree<Data, VecType>*> pGetAtDepth(int depth, std::vector<Octree<Data, VecType>*> vector);

private:
    /**
     * nullptr for root
     */
    Octree<Data, VecType>* m_father { nullptr };
    std::vector<Octree<Data, VecType>*> m_children;
    int m_depth { 0 };
    VecType m_min ;
    VecType m_max ;
    Data m_data;

    /**
     * ambiant space dimension (2 or 3)
     */
    int m_dim;
};
