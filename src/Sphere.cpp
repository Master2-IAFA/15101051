#include "Sphere.hpp"

Sphere::Sphere()
{
  this->center = glm::vec3(0.0f, 0.0f, 0.0f);
  this->radius = 0.0f ;

  this->m_ul = glm::vec3(0.0f, 0.0f, 0.0f);
  this->m_uc = 0.0f ;
  this->m_uq = 0.0f ;
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

   //draw cube for debug
   drawCube("node sphere fitting", octree->getMin(), octree->getMax());

   //group points which are in node for debug
   std::vector<glm::vec3> points_in_cube = delete_points_not_in_cube(octree, ps);
   polyscope::PointCloud *pointCloud = polyscope::registerPointCloud("points in my cube", points_in_cube);

   //display stats of this node
   statistics stat= octree->getData() ;

   //compute u0 u123 u4

   //DANS TOUT CE QUI SUIT WEIGHT ET AREA (LA DENSITE SONT MIT A 1
   auto m_nume = stat.pdn - glm::dot(stat.position, stat.normal) ;
   auto m_deno =  stat.norm * glm::dot(stat.position, stat.position) ;

   float m_uq = 0.5f * m_nume/m_deno ;
   this->m_uq = m_uq ;
   glm::vec3 m_ul = stat.normal - 2.0f * m_uq * stat.position ;
   this->m_ul = m_ul ;
   float m_uc = -1.0f * glm::dot(stat.position, m_ul) + m_uq * stat.norm ;
   this->m_uc = m_uc ;

   //std::cout << m_uq << "\n" ;
   //std::cout << m_ul.x << "," << m_ul.y  << "," << m_ul.z << "\n" ;
   //std::cout << m_uc << "\n" ;

   //check if sphere fits points
   this->center = this->get_center() ;
   this->radius = this->get_radius() ;
   std::cout << "center =" << this->center.x << "," << this->center.y  << "," << this->center.z << "\n" ;
   std::cout << "radius =" << this->radius << "\n" ;
   //display_sphere(0.1f, center) ;

 }
