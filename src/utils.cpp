#include "utils.hpp"

InputOctree *generateInputOctree( int max_depth, PointSet *pc ){
  std::vector<point> aabb = pc->getBoundingBox();
  InputOctree *octree = new InputOctree(0, aabb[0].pos, aabb[1].pos);
  std::vector<point> points = pc->getPoints();
  fitInputOctree( max_depth, octree, &points );
  return octree;
}

void fitInputOctree( int max_depth, InputOctree *octree, std::vector<point> *points ){
  if( max_depth == 0 ) return;

  octree->subDivide();
  auto children = octree->getChildren();

  
  bool hasPoint = false;
  for( int i = 0; i < 8; i++ ){ 

    std::vector<point> children_points;
    statistics stat;
    hasPoint = false;

    for( int j = 0; j < points->size(); j++ ){
      if( children[i]->isPointIn( points->at(j).pos ) ){
        children_points.push_back( points->at(j) );
        //statisticsAdd( &stat, &points[j] );
        hasPoint = true;
      }

    }
    if(hasPoint){
        children[i]->setData( stat );
        fitInputOctree( max_depth - 1, children[i], &children_points );
    }

  }

}

void statisticsAdd(statistics *stat, point point){
    stat->area += 1;
    stat->norm += glm::l2Norm(point.pos);
    stat->normal += point.norm;
    stat->pdn += glm::dot(point.pos, point.norm);
    stat->position += point.pos;
}
