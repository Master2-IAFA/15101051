#include "debug.hpp"

void pointSetToPolyscope( std::string name, PointSet<point3d> *ps ){

    std::vector<point3d> points = ps->getPoints();

    std::vector<glm::vec3> position ( points.size() );
    std::vector<glm::vec3> normal ( points.size() );

    #pragma omp parallel for
    for(int i = 0; i < points.size(); i++){
        position[i] = glm::vec3(points[i].pos[0], points[i].pos[1], points[i].pos[2]) ;
        normal[i] = glm::vec3(points[i].norm[0], points[i].norm[1], points[i].norm[2]);
    }

    polyscope::PointCloud *pointCloud = polyscope::registerPointCloud(name, position);
    pointCloud->addVectorQuantity("normal", normal);
}

void pointSet2dToPolyscope (std::string name, PointSet<point2d> *ps) {
    std::vector<point2d> points = ps->getPoints();

    std::vector<glm::vec2> position ( points.size() );
    std::vector<glm::vec2> normal ( points.size() );

    for ( int i = 0 ; i < points.size() ; ++i ) {
        position[i] = glm::vec2( points[i].pos[0], points[i].pos[1] );
        normal[i] = glm::vec2( points[i].norm[0], points[i].norm[1] );
    }

    polyscope::view::style = polyscope::view::NavigateStyle::Planar;

    polyscope::registerPointCloud2D(name, position);
    polyscope::getPointCloud(name)->addVectorQuantity2D(name + " normal", normal);
}

//function that takes minmax of a cube a returns 8 coordinates of cube
std::vector<glm::vec3> build_cube_from_minmax(glm::vec3 min, glm::vec3 max) {
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

std::vector<glm::vec2> build_square_from_minmax(glm::vec2 min, glm::vec2 max) {
    std::vector<glm::vec2> square;
    square.emplace_back(min);
    square.emplace_back(min[0], max[1]);
    square.emplace_back(max[0], min[1]);
    square.emplace_back(max);

    return square;
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
    PointSet<point3d> p;
    p.readPly("../assets/gaussian_spike_norm.ply");
    polyscope::init();
    pointSetToPolyscope( "gaussian", &p);
    polyscope::show();
}

void test_debug_subdivide () {
    glm::vec3 a (0, 0, 0);
    glm::vec3 b (100, 100, 100);
    Octree<int, glm::vec3>* tree = new Octree<int, glm::vec3>(0, a, b);

    tree->subDivide();

    auto children = tree->getChildren();
    children[0]->subDivide();
    //TODO display tree
}

//-----------------------------VIZUALISE BB OF POINT CLOUD----------------------
void test_debug_bounding_box(std::vector<glm::vec3> points)
{
  //------------------test getBoundingBox (Lou)
  vector<point3d> my_points ;
  std::vector<glm::vec3> colors;
  //glm::vec3 useless_norm = (0.0, 0.0, 0.0);
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

void drawSquare(std::string name, glm::vec2 min, glm::vec2 max) {
    std::vector<std::array<size_t, 2>> edges;
    std::vector<glm::vec2> square = build_square_from_minmax(min, max);

    edges.push_back({0, 1});
    edges.push_back({0, 2});
    edges.push_back({1, 3});
    edges.push_back({2, 3});

    polyscope::registerCurveNetwork(name, square, edges);
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
  //polyscope::show();
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

polyscope::CurveNetwork* drawOctree(std::string name, std::vector<Octree<statistics3d, glm::vec3> *> octree){

  std::vector<std::array<int, 2>> edges ;
  std::vector<glm::vec3> nodes;

  for(int i = 0; i < octree.size(); i++){
    auto min = octree[i]->getMin();
    auto max = octree[i]->getMax();
    auto cube = build_cube_from_minmax( glm::vec3(min[0], min[1], min[2]), glm::vec3(max[0], max[1], max[2]) );
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

polyscope::CurveNetwork* drawQuadtree (std::string name, std::vector<Octree<statistics2d, glm::vec2> *> octree){
    std::vector<std::array<int, 2>> edges ;
    std::vector<glm::vec2> nodes;

    for(int i = 0; i < octree.size(); i++){
        auto min = octree[i]->getMin();
        auto max = octree[i]->getMax();
        auto square = build_square_from_minmax( glm::vec2(min[0], min[1]), glm::vec2(max[0], max[1]) );
        for(int j = 0; j < 4; j++)
            nodes.push_back( square[j] );

        edges.push_back({0 + 4 * i, 1 + 4 * i});
        edges.push_back({2 + 4 * i, 3 + 4 * i});

        edges.push_back({0 + 4 * i, 2 + 4 * i});
        edges.push_back({1 + 4 * i, 3 + 4 * i});
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
