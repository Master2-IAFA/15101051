#pragma once

#include <iostream>
#include <algorithm>
#include <cmath>

/** class that represent an octree define by an aabb box.
 * 
 *  an octree is defined recursively: an octree is either a leaf 
 *  that has no children or it is comprised of 8 children that are 
 *  octrees themselves
 * 
 *  a octree has a label of arbitraty type
 * 
 * @note actually, the term "Octree" has meaning only when talking about
 * 3D data. In 2D, we talk about "Quadtrees"
 * 
 * @tparam Data: Node label type, typically statistics
 */
template<typename Data, typename VecType, typename OctreeType>
class BaseOctree{

public:
    BaseOctree ( OctreeType *_father, int _depth, VecType _min, VecType _max );
    BaseOctree ( int _depth, VecType _min, VecType _max );

    virtual ~BaseOctree ();

    /** split the octree into 8 childrens, and link it as their father. */
    virtual void subDivide ();

    /** get all the octrees at the given depth */
    std::vector<OctreeType*> getAtDepth ( int depth );

    /** check if the given point is inside the octree using it's min/max points */
    bool isPointIn ( VecType p );

    /** return if the octree has children */
    bool hasChildren () { return !(m_children.size() == 0); }

    /***** setters ******/
    void setFather ( OctreeType* _father ) { m_father = _father; }
    inline void setData ( Data& _data ) { m_data = _data; }

    /***** getters ******/
    inline const Data& getData () const { return m_data; }
    inline const VecType& getMin () const { return m_min; }
    inline const VecType& getMax () const { return m_max; }
    inline const int getDepth () const { return m_depth; }
    inline std::vector<OctreeType*>& getChildren () { return m_children; }
    inline const int getDim () const { return m_dim; }
    inline int getMaxDepth();

private:
    /** fetch all nodes that are at depth \p depth */
    std::vector<OctreeType*> pGetAtDepth(int depth, std::vector<OctreeType*> vector);
    void init();

protected:
    /** nullptr for root */
    OctreeType* m_father { nullptr };
    std::vector<OctreeType*> m_children;

    /** maximum depth */
    int m_depth { 0 };

    /** node's bounding box min position value */
    VecType m_min ;
    
    /** node's bounding box max position value */
    VecType m_max ;

    /** typically, m_data countains node statistics */
    Data m_data;

    /** points that are spacially countained in the nod e*/
    std::vector<VecType> m_points;

    /** ambiant space dimension (2 or 3) */
    int m_dim;
};