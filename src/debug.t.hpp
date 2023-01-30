#pragma once
#include "debug.hpp"

/** returns the number of different bits between two integers*/
inline int bitDiff (unsigned int n, unsigned int m) {
    int count = 0;

    do {
        if (n % 2 != m % 2) {
            ++count;
        }
        n /= 2;
        m /= 2;
    } while (n != 0 || m != 0);

    return count;
}

template<class VecType, class PointType>
polyscope::PointCloud* pointSetToPolyscope(std::string name, PointSet<PointType> *ps ){
    std::vector<PointType> points = ps->getPoints();

    std::vector<VecType> position ( points.size() );
    std::vector<VecType> normal ( points.size() );

    #pragma omp parallel for
    for (int i = 0; i < points.size(); ++i) {
        position[i] = points[i].pos;
        normal[i] = points[i].norm;
    }
    polyscope::PointCloud* pointCloud;
    if (points[0].pos.length() == 3)
        pointCloud = polyscope::registerPointCloud(name, position);
    else
        pointCloud = polyscope::registerPointCloud2D(name, position);
    pointCloud->addVectorQuantity("normal", normal);

    return pointCloud ;
}

//function that takes minmax of a cube a returns 8 coordinates of cube
template< class VecType >
std::vector<VecType> build_cube_from_minmax(VecType min, VecType max) {
    std::vector<VecType> cube ;

    for ( int i = 0 ; i < int(pow( 2, min.length() )) ; ++i ) {
        VecType temp = VecType(0);
        for ( int j = 0 ; j < min.length() ; ++j ) {
            if ( (i / int(pow( 2, j ))) % 2 == 0 ) {
                temp[j] = min[j];
            }
            else {
                temp[j] = max[j];
            }
        }
        cube.emplace_back(temp);
    }

    return cube ;
}

void generate_gaussian () {
    std::ofstream file ("gaussian.ply");

    file << "ply" << std::endl;
    file << "format ascii 1.0" << std::endl;
    file << "element vertex " << 50 * 50 << std::endl;
    file << "property float x" << std::endl;
    file << "property float y" << std::endl;
    file << "property float z" << std::endl;
    file << "property float nx" << std::endl;
    file << "property float ny" << std::endl;
    file << "property float nz" << std::endl;
    file << "end_header" << std::endl;

    for(float i = 0; i < 50; i++) {
        for( float j = 0; j < 50; j++) {
            float x = (i - 25) / 25.0f;
            float y = (j - 25) / 25.0f;
            file << i/50.0f << " " << exp(-(x*x) - (y*y)) << " " << j/50.0f
                 << " " <<  2 * x * exp(-(x*x) - (y*y)) << " " <<   j/50.0f << " " <<  2 * y * exp(-(x*x) - (y*y))
                 << std::endl;
        }
    }
    file.close();
}

template<class VecType>
void slide_points(polyscope::PointCloud *pc_init, polyscope::PointCloud * pc_final, int nb_slider_max, int nb_slider)
{
  //get points pos in pc_init and pc_final
  std::vector<VecType> final_pos_list = pc_final->points ;
  std::vector<VecType> init_pos_list = pc_init->points ;

  std::vector<VecType> newPositions ;
  
  if (nb_slider_max == nb_slider)
  {
    pc_init->updatePointPositions(final_pos_list);
  }
  
  else
  {
    for (int i = 0 ; i <  init_pos_list.size() ; ++i)
    {
      //difference between the 2 pos
      VecType diff = final_pos_list.at(i) - init_pos_list.at(i) ;
    
      VecType new_pos = (float(nb_slider)/float(nb_slider_max)) * diff ;
      newPositions.push_back(init_pos_list.at(i) + new_pos);
    }
    pc_init->updatePointPositions(newPositions);
  } 
  //pc_init->updatePointPositions(final_pos_list);
 
}

//-----------------------------VIZUALISE BB OF POINT CLOUD----------------------
void test_debug_bounding_box(std::vector<glm::vec3> points)
{
  //------------------test getBoundingBox (Lou)
  vector<point3d> my_points ;
  std::vector<glm::vec3> colors;
  //VecType useless_norm = (0.0, 0.0, 0.0);
  for (int i = 0 ; i < points.size() ; ++i)
  {
    point3d p;
    p.pos[0] = points[i][0] ;
    p.pos[1] = points[i][1] ;
    p.pos[2] = points[i][2] ;

    p.norm[0] = 1 ;
    p.norm[1] = 0 ;
    p.norm[2] = 0 ;

    my_points.push_back(p);
  }

  PointSet<point3d> * ps = new PointSet(my_points) ;

  std::pair<point3d, point3d> bb_example = ps->getBoundingBox();
  std::cout << "bb_example : min = " << bb_example.first.pos[0] << "," << bb_example.first.pos[1] << "," << bb_example.first.pos[2] ;
  std::cout << "bb_example : max = " << bb_example.second.pos[0] << "," << bb_example.second.pos[1] << "," << bb_example.second.pos[2] ;

  std::vector<glm::vec3> points_bb;
  points_bb.emplace_back(bb_example.first.pos[0], bb_example.first.pos[1], bb_example.first.pos[2]);
  points_bb.emplace_back(bb_example.second.pos[0], bb_example.second.pos[1], bb_example.second.pos[2]);

  polyscope::registerPointCloud("bounding box", points_bb);
  auto f = polyscope::getPointCloud("bounding box");
  f->setEnabled(true);
}

void test_basic_polyscope () {
    std::vector<glm::vec3> points;
    std::vector<glm::vec3> colors;

    polyscope::init();

    // generate points
    for(float i = 0; i < 50; i++){
        for( float j = 0; j < 50; j++){
            float x = (i - 25) / 25.0f;
            float y = (j - 25) / 25.0f;
            points.push_back( glm::vec3( i/50.0f, exp( -(x*x) - (y*y)) , j/50.0f));
            colors.push_back( glm::vec3(0.0f, 0.0f, 0.0f) );
        }
    }
    polyscope::registerPointCloud("gauss", points);
    polyscope::show();
}

template< class VecType >
void drawCube(std::string name, VecType min, VecType max) {
    std::vector<std::array<size_t, 2>> edges ;

    std::vector<VecType> nodes = build_cube_from_minmax<VecType>(min, max);

    for (int i = 0;i < pow(2, min.length());++i) {
        for (int j = i + 1;j < pow(2, min.length());++j) {
            if (bitDiff(i, j) == 1) {
                edges.push_back({i, j});
            }
        }
    }

    //polyscope::init();

    // Add the curve network
    polyscope::registerCurveNetwork(name, nodes, edges);

    // visualize!
    //polyscope::show();
}

template<class VecType>
void draw_diagonal(std::string name, VecType min, VecType max)
{
  std::vector<std::array<size_t, 2>> edges ;
  std::vector<VecType> nodes ;

  nodes.push_back(min) ;
  nodes.push_back(max) ;

  edges.push_back({0, 1});

  polyscope::registerCurveNetwork(name, nodes, edges);
}

template<typename Data, typename VecType, typename OctreeType>
polyscope::CurveNetwork* drawOctree(std::string name, std::vector<BaseOctree<Data, VecType, OctreeType> *> octree){
    std::vector<std::array<int, 2>> edges ;
    std::vector<VecType> nodes;

    for(int i = 0; i < octree.size(); i++){
        auto min = octree[i]->getMin();
        auto max = octree[i]->getMax();
        auto cube = build_cube_from_minmax<VecType>( min , max );
        for(int j = 0; j < pow(2, min.length()); j++)
            nodes.push_back( cube[j] );
        for (int j = 0;j < pow(2, min.length());++j) {
            for (int k = j + 1;k < pow(2, min.length());++k) {
                //double temp = log(k - j) / log(2);
                if (bitDiff(k, j) == 1) {
                    edges.push_back({j + pow(2, min.length()) * i, k + pow(2, min.length()) * i});
                }
            }
        }
    }
    return polyscope::registerCurveNetwork(name, nodes, edges);
}

PointSet<point2d> generate2dGaussian () {
    std::vector<point2d> ps;
    point2d p;

    for( float i = 0 ; i < 50 ; ++i ) {
        for( float j = 0 ; j < 50 ; ++j ) {
            float x = (i - 25) / 25.0f;
            float y = exp( -(x * x) );
            float dy = -2 * x * exp( -(x * x) );

            p.pos = glm::vec2( x, y );
            p.norm = glm::vec2( x, dy );

            ps.emplace_back(p);
        }
    }

    auto pc = PointSet<point2d>(ps);
    return pc;
}

/**
 * @brief This function calculates the traversed node and display only used nodes of the octree.
 *
 */
template< class VecType, class StatType, class PointType >
void _traverse_for_fake_blending (InputOctree<VecType, StatType, PointType> *current_node_octree, VecType& q, std::vector<std::array<int, 2>>* edges, std::vector<VecType>* nodes) {
  if (!current_node_octree->hasChildren())
    return;
  auto children = current_node_octree->getChildren();

  auto cube = build_cube_from_minmax( current_node_octree->getMin(), current_node_octree->getMax());
  int nodes_size = nodes->size();

  for(int j = 0; j < 8; j++)
      nodes->push_back( cube[j] );

    edges->push_back({nodes_size,   nodes_size+1});
    edges->push_back({nodes_size+2, nodes_size+3});
    edges->push_back({nodes_size+4, nodes_size+5});
    edges->push_back({nodes_size+6, nodes_size+7});

    edges->push_back({nodes_size,   nodes_size+2});
    edges->push_back({nodes_size+1, nodes_size+3});
    edges->push_back({nodes_size+4, nodes_size+6});
    edges->push_back({nodes_size+5, nodes_size+7});

    edges->push_back({nodes_size,   nodes_size+4});
    edges->push_back({nodes_size+1, nodes_size+5});
    edges->push_back({nodes_size+2, nodes_size+6});
    edges->push_back({nodes_size+3, nodes_size+7});

  for (int i = 0; i < 8; i++){
      if (children[i]->is_InProtectionSphere( q))
          _traverse_for_fake_blending(children[i], q, edges, nodes);
  }

}

template<typename statistics, typename point, typename VecType, typename PointType>
AlgebraicSphere<VecType, statistics> projection (InputOctree<VecType, statistics, PointType> *octree, float (*kernel)(VecType&,VecType&) ,VecType& q){
    statistics stats = cumul_stats (octree, kernel, q);
    //std::pair<VecType, float> sphere = fit_algebraic_sphere(stats, q, kernel);
    AlgebraicSphere<VecType, statistics> sphere;
    sphere.fitSphere( stats, q, &rational_kernel );
    return sphere;
    // point new_q;
    // new_q.pos = VecType(0.0f);
    // new_q.norm = VecType(0.0f);
    // return new_q;
}

template< class VecType, class StatType, class PointType >
polyscope::PointCloud * draw_traverseOctree_onePoint(InputOctree<VecType, StatType, PointType> * oct, PointSet<point3d> *ps) {

  std::vector<std::array<int, 2>> edges ;
  std::vector<VecType> nodes;

  VecType bb_min = oct->getMin();
  VecType bb_max = oct->getMax();

  VecType bb_mid = ( bb_min + bb_max ) / VecType( 2.0 );
  float radius_protectionSphere = (glm::distance(bb_mid, bb_max) * oct->getProtectionSphere())/2;

  //Creation of a random point into the bounding box.
  float rand_x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/radius_protectionSphere));
  float rand_y = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/radius_protectionSphere));
  float rand_z = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/radius_protectionSphere));
  VecType q = VecType(bb_mid.x+rand_x, bb_mid.y+rand_y, bb_mid.z+rand_z);

  // This pair contains the information of the sphere where we're gonna project the point q : the center and its radius (.first, .second).
  AlgebraicSphere<VecType, statistics3d> sphere = projection<statistics3d, point3d, VecType>(oct, rational_kernel, q);

  //CQFD
  //display_sphere("sphere", sphere.first, sphere.second);

  // Get the projected point.
  VecType projectedPoint = sphere.project( q );

  // Displaying it.
  std::vector<VecType> pc_only_one_point;
  pc_only_one_point.push_back(q);
  pc_only_one_point.push_back(projectedPoint);
  polyscope::PointCloud *pointCloud = polyscope::registerPointCloud("just_a_simple_point", pc_only_one_point);
  pointCloud->setPointRadius(0.02f);

  // Display only nodes that we traverse.
  _traverse_for_fake_blending(oct ,q ,&edges, &nodes);
  polyscope::registerCurveNetwork("Traversing", nodes, edges);

  // Now, we compute the projection with the entire point cloud.
  std::vector<VecType> pc_projected;
  for (auto p : ps->getPoints()){
    VecType current_p = p.pos;
    auto sphere = projection<statistics3d, point3d, VecType>(oct, rational_kernel, current_p);
    VecType current_p_projected = sphere.project( current_p );
    pc_projected.push_back(current_p_projected);
  }
  polyscope::PointCloud *pointCloud_projected = polyscope::registerPointCloud("point_cloud_projected", pc_projected);
  pointCloud_projected->setEnabled(false);
  return pointCloud_projected;
}

template< class VecType >
void display_sphere( std::string name, VecType center, float radius) {
    std::vector<VecType> sphere_pos;

    sphere_pos.push_back( center );

    polyscope::PointCloud *pointCloud = (center.length() == 3)?
              polyscope::registerPointCloud( name, sphere_pos ):
              polyscope::registerPointCloud2D( name, sphere_pos );
    pointCloud->setPointRadius(radius, false);
}


/**
 * @author Léo 
 * 
 * @brief This function take an octree, a depth and an idx for the node at the given depth and show its algebraic sphere. 
 * @param name name of the displayed sphere.
 * @param oct 
 * @param depth Depth of the node we want
 * @param num_child Num of the node for the given depth
 */
template< class VecType, class StatType, class PointType >
void node_stats_to_sphere ( std::string name, InputOctree<VecType, StatType, PointType> * oct, int depth, int num_child){  
    auto octree_depth_three = oct->getAtDepth (depth);

    auto notre_node = (depth == 0)?oct:octree_depth_three.at(num_child);
    auto stat = notre_node->getData();

    float weight_wi = 1.0f;

    VecType pi = weight_wi * stat.position;
    VecType ni = weight_wi * stat.normal;
    float area = weight_wi * stat.area;
    float pi_ni = weight_wi * stat.pdn;
    float norm = weight_wi * stat.norm;

    float num = pi_ni - glm::dot( pi, ni )/area;
    float denom = norm - ( glm::dot( pi, pi ) / area );
    float m_u4 = ( num / denom ) / 2;

    VecType num_vec = ni - VecType( 2.0 ) * VecType( m_u4 ) * pi ;
    VecType m_u123 = num_vec / area;

    auto num_2 = glm::dot( pi, m_u123 ) + m_u4 * norm;
    float m_u0 = - num_2 / area;

    float b = 1.0f / m_u4;
    VecType m_center = -0.5f * m_u123 * b;

    b = m_u0 / m_u4;
    auto cTc = glm::dot( m_center, m_center );
    double r = sqrt( cTc - b );
    float m_radius = std::max( 0.0 , r );

    std::vector<VecType> sphere_pos;

    sphere_pos.push_back( m_center );

    drawCube<VecType>("Le cube", notre_node->getMin(), notre_node->getMax());

    polyscope::PointCloud *pointCloud = polyscope::registerPointCloud( name, sphere_pos );
    pointCloud = polyscope::registerPointCloud( name, sphere_pos );
    pointCloud->setPointRadius(m_radius, false);
}

/**
 * @author Léo 
 * 
 * @brief This function takes a point and its fitted algebraic sphere to display the point, the sphere and its projection. 
 * @param name name of the displayed sphere.
 * @param point point that we want to fit a sphere.
 * @param sphere fitted sphere.
 */
template< class VecType, class StatType >
void point_and_stats_to_sphere (std::string point_name, std::string name, VecType point, VecType end, AlgebraicSphere<VecType, StatType> sphere){
    std::vector<VecType> pos;
    std::vector<VecType> pos_sphere;

    pos.push_back(point);
    pos.push_back(end);
    pos_sphere.push_back(sphere.getCenter());

    polyscope::PointCloud *pc_point = (point.length() == 3)?
                polyscope::registerPointCloud( point_name, pos ):
                polyscope::registerPointCloud2D( point_name, pos );

    pc_point->setPointRadius(0.02);
    polyscope::PointCloud *pc_sphere = (point.length() == 3)?
                polyscope::registerPointCloud( name, pos_sphere ):
                polyscope::registerPointCloud2D( name, pos_sphere );
    pc_sphere->setPointRadius(sphere.getRadius(), false);
}
