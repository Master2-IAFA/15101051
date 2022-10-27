#include "debug.hpp"

void pointSetToPolyscope( std::string name, PointSet *ps ){

    std::vector<point> points = ps->getPoints();

    std::vector<glm::vec3> position(points.size());
    std::vector<glm::vec3> normal(points.size());

    #pragma omp parallel for
    for(int i = 0; i < points.size(); i++){
        position[i] = points[i].pos;
        normal[i] = points[i].norm;
    }

    polyscope::PointCloud *pointCloud = polyscope::registerPointCloud(name, position);
    pointCloud->addVectorQuantity("normal", normal);
}

//function that takes minmax of a cube a returns 8 coordinates of cube
std::vector<glm::vec3> build_cube_from_minmax(glm::vec3 min, glm::vec3 max)
{
  //float len_body_diag = (max-min).length() ;
  //float a = len_body_diag/sqrt(3) ;

  glm::vec3 tfl = glm::vec3(min.x, max.y, min.z ) ;
  glm::vec3 tfr = glm::vec3(max.x, max.y, min.z) ;
  glm::vec3 tbl = glm::vec3(min.x, max.y, max.z ) ;
  glm::vec3 tbr = glm::vec3(max.x, max.y, max.z) ;
  glm::vec3 bfl = glm::vec3(min.x, min.y, min.z) ;
  glm::vec3 bfr =glm::vec3(max.x, min.y, min.z) ;
  glm::vec3 bbl  = glm::vec3(min.x, min.y, max.z) ;
  glm::vec3 bbr = glm::vec3(max.x, min.y, max.z) ;

  std::vector<glm::vec3> cube ;
  cube.push_back(tfl);
  cube.push_back(tfr);
  cube.push_back(tbl);
  cube.push_back(tbr);
  cube.push_back(bfl);
  cube.push_back(bfr);
  cube.push_back(bbl);
  cube.push_back(bbr);
  return cube ;

}

void generate_gaussian () {
    std::ofstream file ("gaussian_spike_norm.ply");

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
            file << i/50.0f << " " << exp(-(x*x) - (y*y)) << " " << j/50.0f << " " << 1.0 << " " << 0.0 << " " << 0.0 << std::endl;
        }
    }
    file.close();
}

void test_debug_readPly () {
    PointSet p;
    p.readPly("../assets/gaussian_spike_norm.ply");
    polyscope::init();
    pointSetToPolyscope( "gaussian", &p);
    polyscope::show();
}

void test_debug_subdivide () {
    Octree<int> tree (0, glm::vec3(0, 0, 0), glm::vec3(100, 100, 100));

    tree.subDivide();

    auto children = tree.getChildren();
    children[0]->subDivide();
    //TODO display tree
}

void _traverse_for_fake_blending (InputOctree* current_node_octree, glm::vec3& q, std::vector<std::array<int, 2>>* edges, std::vector<glm::vec3>* nodes) {
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
      if (is_InProtectionSphere(children[i], q))
          _traverse_for_fake_blending(children[i], q, edges, nodes);
  }

  
}

void draw_traverseOctree_onePoint(InputOctree* oct) {

  std::vector<std::array<int, 2>> edges ;
  std::vector<glm::vec3> nodes;

  glm::vec3 bb_min = oct->getMin();
  glm::vec3 bb_max = oct->getMax();

  glm::vec3 bb_mid = midpoint(bb_min, bb_max);
  float radius_protectionSphere = (glm::distance(bb_mid, bb_max) * LAMBDA)/2;

  float rand_x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/radius_protectionSphere));
  float rand_y = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/radius_protectionSphere));
  float rand_z = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/radius_protectionSphere));

  glm::vec3 q = glm::vec3(bb_mid.x+rand_x, bb_mid.y+rand_y, bb_mid.z+rand_z);
  std::vector<glm::vec3> pc_only_one_point;
  pc_only_one_point.push_back(q); 
  polyscope::PointCloud *pointCloud = polyscope::registerPointCloud("just_a_simple_point", pc_only_one_point);
  pointCloud->setPointRadius(0.02f);

  _traverse_for_fake_blending(oct ,q ,&edges, &nodes);

  polyscope::registerCurveNetwork("Traversing", nodes, edges);

}

//-----------------------------VIZUALISE BB OF POINT CLOUD----------------------
void test_debug_bounding_box(std::vector<glm::vec3> points)
{
  //------------------test getBoundingBox (Lou)
  vector<point> my_points ;
  std::vector<glm::vec3> colors;
  //glm::vec3 useless_norm = (0.0, 0.0, 0.0);
  for (int i = 0 ; i < points.size() ; ++i)
  {
    point p;
    p.pos = points[i] ;
    p.norm = glm::vec3(0.0, 0.0, 0.0) ;
    my_points.push_back(p);
  }

  PointSet * ps = new PointSet(my_points) ;
  vector<point> bb_example ;

  bb_example = ps->getBoundingBox();
  std::cout << "bb_example : min = " << bb_example[0].pos.x << "," << bb_example[0].pos.y << "," << bb_example[0].pos.z ;
  std::cout << "bb_example : max = " << bb_example[1].pos.x << "," << bb_example[1].pos.y << "," << bb_example[1].pos.z ;

  std::vector<glm::vec3> points_bb;
  for (int i = 0 ; i < bb_example.size() ; ++i)
  {
    points_bb.push_back(bb_example[i].pos) ;
  }
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

void drawCube(std::string name, glm::vec3 min, glm::vec3 max)
{
  std::vector<std::array<size_t, 2>> edges ;

  std::vector<glm::vec3> nodes = build_cube_from_minmax(min, max);

  edges.push_back({0, 1});
  edges.push_back({2, 3});
  edges.push_back({4, 5});
  edges.push_back({6, 7});

  edges.push_back({0, 2});
  edges.push_back({1, 3});
  edges.push_back({4, 6});
  edges.push_back({5, 7});

  edges.push_back({0, 4});
  edges.push_back({1, 5});
  edges.push_back({2, 6});
  edges.push_back({3, 7});

  //polyscope::init();

  // Add the curve network
  polyscope::registerCurveNetwork(name, nodes, edges);

  // visualize!
  // polyscope::show();
}

void draw_diagonal(std::string name, glm::vec3 min, glm::vec3 max)
{
  std::vector<std::array<size_t, 2>> edges ;
  std::vector<glm::vec3> nodes ;

  nodes.push_back(min) ;
  nodes.push_back(max) ;

  edges.push_back({0, 1});

  polyscope::registerCurveNetwork(name, nodes, edges);
}

polyscope::CurveNetwork* drawOctree(std::string name, std::vector<InputOctree *> octree){

  std::vector<std::array<int, 2>> edges ;
  std::vector<glm::vec3> nodes;

  for(int i = 0; i < octree.size(); i++){

    auto cube = build_cube_from_minmax( octree[i]->getMin(), octree[i]->getMax());
    for(int j = 0; j < 8; j++)
      nodes.push_back( cube[j] );

    edges.push_back({0 + 8 * i, 1 + 8 * i});
    edges.push_back({2 + 8 * i, 3 + 8 * i});
    edges.push_back({4 + 8 * i, 5 + 8 * i});
    edges.push_back({6 + 8 * i, 7 + 8 * i});

    edges.push_back({0 + 8 * i, 2 + 8 * i});
    edges.push_back({1 + 8 * i, 3 + 8 * i});
    edges.push_back({4 + 8 * i, 6 + 8 * i});
    edges.push_back({5 + 8 * i, 7 + 8 * i});

    edges.push_back({0 + 8 * i, 4 + 8 * i});
    edges.push_back({1 + 8 * i, 5 + 8 * i});
    edges.push_back({2 + 8 * i, 6 + 8 * i});
    edges.push_back({3 + 8 * i, 7 + 8 * i});
  }
  return polyscope::registerCurveNetwork(name, nodes, edges);

}


