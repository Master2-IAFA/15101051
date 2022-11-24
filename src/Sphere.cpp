#include "Sphere.hpp"


/**
 * @brief Kernel used into the projection process it takes 2 points and return a scalar
 * @param p : A point that will be compared with q
 * @param q : A point that will be compared with p
 */
float rational_kernel (glm::vec3& p, glm::vec3& q){
    // Interpolation : Epsilon = 0.0f
    // approximatin surfaces : Epsilon > 0.0f
    float epsilon = 0.5f;
    float dist = glm::pow(glm::distance(p, q),2);

    // Try to find a good k
    int k = 1;

    float res = glm::pow((dist + epsilon), (-k/2));
    return res;
}

Sphere::Sphere()
{
  this->center = glm::vec3(0.0f, 0.0f, 0.0f);
  this->radius = 0.0f ;

  this->m_ul = glm::vec3(0.0f, 0.0f, 0.0f);
  this->m_uc = 0.0f ;
  this->m_uq = 0.0f ;
}

glm::vec3 get_center(glm::vec3 m_ul, float m_uq)
{
    float b = 1.0f/m_uq;
    //return - (m_ul / 2.0f); //+ Base::m_w.basisCenter();
    return - (m_ul / glm::vec3(2.0));
}


float Sphere::get_radius()
 {
     float b = 1.0f/ this->m_uq;
     this->radius = sqrt(glm::pow(glm::l2Norm((-0.5f*b)*this->m_ul), 2) - this->m_uc*b );
     return this->radius ;
 }


 glm::vec3 Sphere::get_center()
 {
     float b = 1.0f/this->m_uq;
     this->center = ((-0.5f)*b)*this->m_ul ;
     return this->center ;
 }

 void Sphere::fit_sphere_on_node(InputOctree * octree, PointSet * ps, glm::vec3 q)
{
  //draw q for debug
  display_sphere(0.02, q);

  //draww cube for debug
  drawCube("node sphere fitting", octree->getMin(), octree->getMax());

  //group points which are in node for debug
  std::vector<glm::vec3> points_in_cube = delete_points_not_in_cube(octree, ps);
  polyscope::PointCloud *pointCloud = polyscope::registerPointCloud("points in my cube", points_in_cube);

  //display stats of this node
  statistics stat = octree->getData() ;
  //auto data =  ;
  std::cout << "area " << stat.area << "\n"; //sigma
  std::cout << "norm " << stat.norm << "\n" ; //pbeta
  std::cout << "pdn " << stat.pdn << "\n"; //pn_beta  = n_sum_dot_pn
  std::cout << "normal " << stat.normal.x << " " << stat.normal.y << " " << stat.normal.z <<"\n";
  std::cout << "position " << stat.position.x << " " << stat.position.y << " " << stat.position.z << "\n";

  float invSumW = 1.0 / stat.area; //TODO remplacer stat.area par la sum des poids (kernel)
  std::cout << invSumW << std::endl;
  //DANS TOUT CE QUI SUIT WEIGHT ET AREA (LA DENSITE) SONT MIT A 1
  auto m_nume = stat.pdn - invSumW * glm::dot(stat.position, stat.normal) ;
  auto m_deno = stat.norm - invSumW * glm::dot(stat.position, stat.position) ;

  auto m_uq = 0.5f * m_nume / m_deno ;
  glm::vec3 m_ul = ( stat.normal - stat.position * glm::vec3(2.0) * m_uq ) * glm::vec3(invSumW);
  auto m_uc = - invSumW * ( glm::dot( stat.position, m_ul ) + stat.norm * m_uq );
  std::cout << m_uq << "\n" ;
  std::cout << m_ul.x << "," << m_ul.y  << "," << m_ul.z << "\n" ;
  std::cout << m_uc << "\n" ;

  this->m_uq = m_uq ;
  this->m_uc = m_uc ;
  this->m_ul = m_ul ;

  //check if sphere fits points
  this->center = get_center() ;
  this->radius = get_radius() ;
  std::cout << "center =" << this->center.x << "," << this->center.y  << "," << this->center.z << "\n" ;
  std::cout << "radius =" << radius << "\n" ;
  //display_sphere(radius, center) ;

 }
