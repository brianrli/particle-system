// Source file for the R3 sphere class 



// Include files 

#include "R3.h"
#include <iostream>
#include <cfloat>


// Public variables 

const R3Sphere R3null_sphere(R3Point(0.0, 0.0, 0.0), -1.0);
const R3Sphere R3zero_sphere(R3Point(0.0, 0.0, 0.0), 0.0);
const R3Sphere R3unit_sphere(R3Point(0.0, 0.0, 0.0), 1.0);

bool R3Sphere::
Intersect(R3Ray& ray, double t0, double& t_final,R3Vector& N)
{

  // R3Vector d = ray.Vector();
  // d.Normalize();
  R3Vector d = -R3Vector(0,0,1);
  R3Point p0 = ray.Start();
  R3Point pc = Center();
  R3Point ipoint;

  // R3Vector d = R3Vector(0,0,1);
  // d = -d;
  // R3Point p0 = R3Point(5,5,-3);
  // R3Point pc = Center();

  double a = d.Dot(d);
  double b = (2*d).Dot(p0-pc);
  double c = ((p0-pc).Dot(p0-pc))-pow(Radius(),2);

  double discriminant = (b*b)-(4*a*c);

  // std::cout << "sphere intersect info " << discriminant << "\n";
  // std::cout << "a " << a << "\n";
  // std::cout << "b " << b << "\n";
  // std::cout << "c " << c << "\n";
  // std::cout <<  "t1 " << (b+sqrt(discriminant))/(2*a) << "\n"; 
  // std::cout <<  "t2 " << (b-sqrt(discriminant))/(2*a) << "\n"; 

  // while(1)
  //   int b = 0;
  
  if(discriminant<0) 
  {
    return false; //none
  }
  
  double t2 = (b+sqrt(discriminant))/(2*a); //exit
  if(t2<0) return false;

  double t1 = (b-sqrt(discriminant))/(2*a);

  if(t1>0 && t0>t1){
    t_final = t1;
    ipoint = ray.Point(t1);
  }
  else if (t0>t2){
    t_final = t2;
    ipoint = ray.Point(t2);
  }
  else
    return false;

  //intersection found
  N = ipoint-Center();
  std::cout << "sphere intersect info " << discriminant << "\n";
  std::cout << "a " << a << "\n";
  std::cout << "b " << b << "\n";
  std::cout << "c " << c << "\n";
  // std::cout << "b2 " << pow(b,2) << "\n";
  // std::cout << "4ac " << (4*a*c) << "\n";
  // R3Point drake = ray.Point(1.0);
  // std::cout << " " << ipoint[0] << " " << ipoint[1] << " " << ipoint[2] <<"\n"; 
  // std::cout << " " << drake[0] << " " << drake[1] << " " << drake[2] <<"\n"; 
  std::cout << " " << p0[0] << " " << p0[1] << " " << p0[2] <<"\n"; 
  std::cout << " " << d[0] << " " << d[1] << " " << d[2] <<"\n"; 
  std::cout  << t0 << " " << t1 << " " << t2 << "\n";
  N.Normalize();
  return true;
}

R3Sphere::
R3Sphere(void)
{
}



R3Sphere::
R3Sphere(const R3Sphere& sphere)
  : center(sphere.center),
    radius(sphere.radius)
{
}



R3Sphere::
R3Sphere(const R3Point& center, double radius)
  : center(center),
    radius(radius)
{
}



double R3Sphere::
Area(void) const
{
  // Return surface area of sphere
  return (4.0 * M_PI * radius * radius);
}



double R3Sphere::
Volume(void) const
{
  // Return volume of sphere
  return (1.3333333333333 * M_PI * radius * radius * radius);
}



R3Box R3Sphere::
BBox(void) const
{
  // Return bounding box of sphere
  return R3Box(center.X() - radius, center.Y() - radius, center.Z() - radius,
               center.X() + radius, center.Y() + radius, center.Z() + radius);
}



void R3Sphere::
Empty(void)
{
  // Empty sphere
  *this = R3null_sphere;
}



void R3Sphere::
Translate(const R3Vector& vector)
{
  // Move sphere center
  center.Translate(vector);
}



void R3Sphere::
Reposition(const R3Point& center)
{
  // Set sphere center
  this->center = center;
}



void R3Sphere::
Resize(double radius) 
{
  // Set sphere radius
  this->radius = radius;
}


void R3Sphere::
Draw(void) const
{
  // Draw sphere
  glPushMatrix();
  glTranslated(center[0], center[1], center[2]);
  static GLUquadricObj *glu_sphere = gluNewQuadric();
  gluQuadricTexture(glu_sphere, GL_TRUE);
  gluQuadricNormals(glu_sphere, (GLenum) GLU_SMOOTH);
  gluQuadricDrawStyle(glu_sphere, (GLenum) GLU_FILL);
  gluSphere(glu_sphere, radius, 32, 32);
  glPopMatrix();
}



void R3Sphere::
Outline(void) const
{
  // Draw sphere in wireframe
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  Draw();
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}



void R3Sphere::
Print(FILE *fp) const
{
  // Print min and max points
  fprintf(fp, "(%g %g %g) %g", center[0], center[1], center[2], radius);
}

