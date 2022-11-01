#include "debug.hpp"

std::vector<glm::vec3> delete_points_not_in_cube(InputOctree * octree, PointSet * ps)
{
  vector<point> points = ps->getPoints() ;
  std::vector<glm::vec3> points_in_cube ;
  for (int i = 0 ; i < points.size() ; ++i)
  {
    if (octree->isPointIn(points[i].pos))
    {
      points_in_cube.push_back(points[i].pos);
    }
  }
  return points_in_cube ;
}

void fit_sphere_on_node(InputOctree * octree, PointSet * ps, glm::vec3 q)
{
  //draw q for debug
  display_sphere(0.02, q);

  //draww cube for debug
  drawCube("node sphere fitting", octree->getMin(), octree->getMax());

  //group points which are in node for debug
  std::vector<glm::vec3> points_in_cube = delete_points_not_in_cube(octree, ps);
  polyscope::PointCloud *pointCloud = polyscope::registerPointCloud("points in my cube", points_in_cube);

  //display stats of this node
  statistics stat= octree->getData() ;
  //auto data =  ;
  //std::cout << "area " << stat.area << "\n"; //sigma
  //std::cout << "norm " << stat.norm << "\n" ; //pbeta
  //std::cout << "pdn " << stat.pdn << "\n"; //pn_beta  = n_sum_dot_pn
  //stat->normal += point.norm; //n alpha
  //std::cout << "position " << stat.position.x << stat.position.y << stat.position.z << "\n"; //p alpha
  //m_nume
  //w = kernel

  //compute algebraic sphere

  /*stat->area += 1;
  stat->norm += glm::l2Norm(point.pos);
  stat->normal += point.norm;
  stat->pdn += glm::dot(point.pos, point.norm);
  stat->position += point.pos;*/

  //DANS TOUT CE QUI SUIT WEIGHT ET AREA SONT MIT A 1
  auto m_nume = stat.pdn - glm::dot(stat.position, stat.normal) ;
  auto m_deno =  stat.norm * glm::dot(stat.position, stat.position) ;

  auto m_uq = 0.5f * m_nume/m_deno ;
  auto m_ul = stat.normal - 2.0f * m_uq * stat.position ;
  auto m_uc = -1.0f * glm::dot(stat.position, m_ul) + m_uq * stat.norm ;

  //std::cout << m_uq << "\n" ;
  //std::cout << m_ul.x << "," << m_ul.y  << "," << m_ul.z << "\n" ;
  //std::cout << m_uc << "\n" ;

  //check if sphere fits points
  glm::vec3 center = get_center( m_ul, m_uq) ;
  float radius = get_radius(m_ul, m_uc, m_uq) ;
  std::cout << "center =" << center.x << "," << center.y  << "," << center.z << "\n" ;
  std::cout << "radius =" << radius << "\n" ;
  //display_sphere(0.1f, center) ;

}


void display_sphere(float radius, glm::vec3 center)
{
  std::vector<glm::vec3> sphere_pos ;
  sphere_pos.push_back(center) ;

  polyscope::PointCloud *pointCloud = polyscope::registerPointCloud("sphere", sphere_pos);
  pointCloud->setPointRadius(radius);

}

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

/*!
       \brief return the estimated radius of the sphere
       \warning return inf if the fitted surface is planar
       \ATTENTION LE CAS OU ON A UN PLAN EST IGNORE (VOIR L150 DE
        ALGEBRAIC SPHERE DE PONCA)
   */
  float get_radius(glm::vec3 m_ul, float m_uc, float m_uq)
   {
       float b = 1.0f/m_uq;
       //glm::pow(glm::l2Norm(point.pos)
        //return Scalar(sqrt( ((Scalar(-0.5)*b)*m_ul).squaredNorm() - m_uc*b ));
        return sqrt(glm::pow(glm::l2Norm((-0.5f*b)*m_ul), 2) - m_uc*b );
   }

   /*!
       \brief return the estimated center of the sphere
       \ATTENTION LE CAS OU ON A UN PLAN EST IGNORE (VOIR L150 DE
       \ALGEBRAIC SPHERE DE PONCA) ET IL MANQUE LA COMPOSANTE BASIS CENTER
       \DANS LE CALCUL

   */
   glm::vec3 get_center(glm::vec3 m_ul, float m_uq)
   {
       float b = 1.0f/m_uq;
       return ((-0.5f)*b)*m_ul ; //+ Base::m_w.basisCenter();
   }
