// Source file for the particle system

// Include files
#include "R2/R2.h"
#include "R3/R3.h"
#include "R3Scene.h"
#include "particle.h"

//debugging
#include <iostream>




////////////////////////////////////////////////////////////
// Random Number Generator
////////////////////////////////////////////////////////////

static double 
RandomNumber(void) 
{
#if defined(_WIN32)
  int r1 = rand();
  double r2 = ((double) rand()) / ((double) (RAND_MAX + 1));
  return (r1 + r2) / ((double) (RAND_MAX + 1));
#else
  return drand48();
#endif
}



////////////////////////////////////////////////////////////
// Generating Particles
////////////////////////////////////////////////////////////

void GenerateParticles(R3Scene *scene, double current_time, double delta_time)
{
  // Generate new particles for every source

  // FILL IN CODE HERE
  vector<R3ParticleSource *>::iterator ps_iter;
  for(ps_iter = scene->particle_sources.begin(); 
    ps_iter != scene->particle_sources.end();
    ++ps_iter)
  {
    R3ShapeType type = (*ps_iter)->shape->type; 
    R3Point position;
    R3Vector direction;
    bool success = false;
    switch(type){
      
      case R3_SPHERE_SHAPE:
      {
        //position
        R3Sphere *sphere = (*ps_iter)->shape->sphere;
        double z = (RandomNumber()*2*sphere->Radius())-sphere->Radius();
        double phi = M_PI * RandomNumber() * 2;
        double d = sqrt(pow(sphere->Radius(),2)-pow(z,2));
        double px = sphere->Center().X() + d * cos(phi);
        double py = sphere->Center().Y() + d * sin(phi);
        double pz = sphere->Center().Z() + z;
        position = R3Point(px,py,pz);

        //direction
        direction = R3Vector(px-sphere->Center().X(),
          py-sphere->Center().Y(),
          pz-sphere->Center().Z());
        direction.Normalize();
        direction *= (*ps_iter)->velocity;

        success = true;
        // std::cout << cos(phi) << " " << sin(phi) << "\n";
        // std::cout << z << " " << phi << " " << d << "\n";
        // std::cout << px << " " << py << " " << pz << "\n";
        break;
      }
      
      case R3_CIRCLE_SHAPE:
      {
        //=====[ position ]=====
        R3Circle *circle = (*ps_iter)->shape->circle;
        R3Vector N = circle->Normal();
        N.Normalize();

        R3Vector A = R3Vector(0,0,0); 
        A[N.MinDimension()] = 1;
        A.Normalize();
        
        A.Cross(N); //perpendicular vector
        A.Normalize();
        double phi = RandomNumber()*2*M_PI;
        double d = circle->Radius() * RandomNumber();
        A.Rotate(N,phi);
        A*=d;
        position = A+circle->Center();

        //=====[ position ]=====
        double t1 = RandomNumber()*M_PI*2;
        double t2 = RandomNumber()*
        sin((*ps_iter)->angle_cutoff);
        A.Normalize();
        
        
        R3Vector V = A;
        V.Rotate(N,t1);
        R3Vector V1 = V;
        V1.Cross(N);
        V.Rotate(V1,acos(t2));
        V.Normalize();

        // std::cout << V.X() << " " << V.Y() << " " << V.Z() << "\n";
        // std::cout << W.X() << " " << W.Y() << " " << W.Z() << "\n";
        // std::cout << v.X() << " " << v.Y() << " " << v.Z() << "\n";

        direction = V * (*ps_iter)->velocity;
      
        success=true;
        break;
      }
      
      default:
        break;
    }

    if(success){
      R3Particle *particle = new R3Particle();
      particle->position = position;
      particle->velocity = direction;
      particle->mass = (*ps_iter)->mass;
      particle->fixed = (fixed) ? true : false;
      particle->drag = (*ps_iter)->drag;
      particle->elasticity = (*ps_iter)->elasticity;
      particle->lifetime = (*ps_iter)->lifetime;
      particle->material = (*ps_iter)->material;   
      
      scene->particles.push_back(particle);  
    }
  


    // Add particle to scene
    // std::cout << (*ps_iter)->rate << "\n";
  }

}



////////////////////////////////////////////////////////////
// Updating Particles
////////////////////////////////////////////////////////////

void UpdateParticles(R3Scene *scene, double current_time, double delta_time, int integration_type)
{
  // Update position for every particle

  // FILL IN CODE HERE
  //for every particle
  bool deleted = false;

  for (int i = 0; i < scene->NParticles(); i++) {
    R3Particle *particle = scene->Particle(i);
    R3Vector force = R3Vector(0,0,0); 

    // GRAVITY
    force += particle->mass * scene->gravity;

    // DRAG
    force += -particle->drag * particle->velocity;

    // SINK
    for (int j = 0; j < scene->NParticleSinks(); j++) {
      R3ParticleSink *sink = scene->ParticleSink(j);
      //spheres
      if(sink->shape->type == R3_SPHERE_SHAPE){
        R3Sphere *sphere = sink->shape->sphere;
        R3Vector to_sink = sphere->Center()-particle->position;
        double d = to_sink.Length() - sphere->Radius();
        
        if(d > 0){
          double ca = sink->constant_attenuation;
          double la = sink->linear_attenuation;
          double qa = sink->quadratic_attenuation;
          to_sink.Normalize();
          to_sink *= ((sink->intensity)/(ca + (la*d) + qa*(pow(d,2))));
          force+= to_sink;
        }
        else{
          //delete
          scene->particles.erase(scene->particles.begin() + i);
          deleted = true;
          break;
        }
      }
    }

    //forward Euler integration
    if(!deleted){

      bool intersect = false;
      //PARTICLE COLLISION TEST
      R3Point old_point = particle->position;
      // R3Vector v = (old_point + (particle->velocity * delta_time))-old_point;
     

      R3Node *node = scene->root;
      vector<R3Node *>::iterator n_iter;
      
      for(n_iter = node->children.begin(); n_iter != node->children.end();++n_iter){
        
        R3ShapeType type = (*n_iter)->shape->type;
        
        switch(type){

          //SPHERE COLLISIONS
          case R3_SPHERE_SHAPE :
          {
            R3Vector v = particle->velocity * delta_time; 
            R3Ray ray = R3Ray(old_point,v);

     				R3Sphere *sphere = (*n_iter)->shape->sphere;
	          double t;
            R3Vector N;

            //INTERSECTION
	          if(sphere->Intersect(ray,v.Length(),t,N))
            {
              R3Vector vel = particle->velocity;
              // v.Normalize();
              // v*=t; //get moment of intersection
              std::cout << " " << vel[0] << " " << vel[1] << " " << vel[2] <<"\n";
              // std::cout << " " << v[0] << " " << v[1] << " " << v[2] <<"\n";
              R3Vector ref = vel - (2*vel.Dot(N)*N);
              // std::cout << " " << ref[0] << " " << ref[1] << " " << ref[2] <<"\n";
              ref[N.MaxDimension()] *= particle->elasticity;
              std::cout << " " << ref[0] << " " << ref[1] << " " << ref[2] <<"\n";
              particle->velocity = ref;

              intersect = true;
            }
          }
          
          default:
            break;
        }

      }

      R3Point pp = particle->position;
      if(intersect)
        std::cout << " " << pp[0] << " " << pp[1] << " " << pp[2] <<"\n"; 
      
      particle->position = particle->position + (particle->velocity * delta_time);    
      pp = particle->position;
      
      if(intersect){
        std::cout << " " << pp[0] << " " << pp[1] << " " << pp[2] <<"\n"; 
        std::cout << "***************************\n";
      }

      particle->velocity = particle->velocity + (delta_time * (force/particle->mass));

      //INTERSECT
      // R3Ray(const R3Point& point, const R3Vector& vector, bool normalized = false);
    }
    else{
      i--;
      delete particle;
      deleted=false;
    }
    
    // std::cout << particle->position[0] << particle->position [1] << particle->position [2] << "\n";
  }
}



////////////////////////////////////////////////////////////
// Rendering Particles
////////////////////////////////////////////////////////////

void RenderParticles(R3Scene *scene, double current_time, double delta_time)
{  
  // REPLACE CODE HERE
  glDisable(GL_LIGHTING);
  glPointSize(5);
  glBegin(GL_POINTS);
  for (int i = 0; i < scene->NParticles(); i++) {
    R3Particle *particle = scene->Particle(i);
    glColor3d(particle->material->kd[0], particle->material->kd[1], particle->material->kd[2]);
    const R3Point& position = particle->position;
    glVertex3d(position[0], position[1], position[2]);
  }   
  glEnd();
}




