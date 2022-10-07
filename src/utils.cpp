#include "utils.hpp"



/*InputOctree *generateOctree( int max_depth, PointSet *pc ){
  std::vector<point> aabb = pc->getBoundingBox();
  InputOctree octree = new InputOctree(aabb[0].pos, aabb[1].pos);
  fitOctree( max_depth, octree, pc );
  return octree;
}

void fitOctree( int max_depth, InputOctree *octree , PointSet *pc ){
  if(max_depth == 0) return;

  octree->subDivise();
  auto children = octree->getChildren();
  auto points = pc->getPoints();

  for(int i = 0; i < 8; i++){
    for( int j = 0; j < points.size(); j++ ){
      if( children[i]->isPointIn(points.at(j).pos) ){
        fitOctree( max_depth - 1, children[i], pc);
        break;
      }
    }
  }

}*/
