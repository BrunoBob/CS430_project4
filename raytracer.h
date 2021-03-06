#ifndef __RAY_TRACER
#define __RAY_TRACER

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define ERROR_RAYCAST 2
#define ERROR_WRITING 3

#define EPSILON 0.01
#define LEVEL_MAX_SHADE 5

typedef struct object{
  int kind; // 0 = sphere, 1 = plane
  double* diffuseColor;
  double* specularColor;
  double* position;
  double reflectivity;
  double refractivity;
  double ior;
  union {
    struct {
      double radius;
    } sphere;
    struct {
      double* normal;
    } plane;
  };
  struct object* next;
} *objectList;

typedef struct light{
  double* color;
  double* position;
  double* direction;
  double radA0, radA1, radA2, angA0, theta;
  struct light* next;
} *lightList;

typedef struct component{
  objectList objects;
  lightList lights;
} *components;

void printObjects(objectList list);

void printLights(lightList list);

double shoot(double* Ro, double* Rd, objectList object);

double* getRefractedRay(double* N, double ior1, double ior2, double* Rd);

double* shade(lightList light, objectList allObject, objectList object, double* Ro, double* Rd, double bestT, int level, double ior);

double* directShade(double* color, lightList light, objectList object, double* Rdn, double* Rd, double* Vo, double* Ron, double dist);

void createScene(char* ppm, unsigned char* data, int width, int height);

double planeIntersection(double* Ro, double* Rd, double* position, double* normal);

double sphereIntersection(double* Ro, double* Rd, double* position, double radius);

double fAng(double* Vo, double* Vl, double angleMax, double a0);

double fRad(double dist, double a0, double a1, double a2);

double* diffuse(double* objDiffuse, double* lightColor, double* N, double* L);

double* specular(double* objSpecular, double* lightColor, double* R, double* V,  double* N, double* L, double shininess);

static inline double* getVector(double x, double y, double z){
  double* v = malloc(3*sizeof(double));
  v[0] = x;
  v[1] = y;
  v[2] = z;
  return v;
}

static inline double* subVector(double* a, double* b){
  return getVector(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

static inline double* addVector(double* a, double* b){
  return getVector(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
}

static inline double* multVector(double* a, double* b){
  return getVector(a[0] * b[0], a[1] * b[1], a[2] * b[2]);
}

static inline double* scaleVector(double* a, double b){
  return getVector(a[0] * b, a[1] * b, a[2] * b);
}

static inline double dotProduct(double* a, double* b){
  return (a[0]*b[0] + a[1]*b[1] + a[2]*b[2]);
}

static inline double sqr(double v) {
  return v*v;
}

static inline double norm(double* a){
  return sqrt((a[0] * a[0]) + (a[1] * a[1]) * (a[2] * a[2]));
}

static inline void normalize(double* v) {
  double len = sqrt(sqr(v[0]) + sqr(v[1]) + sqr(v[2]));
  v[0] /= len;
  v[1] /= len;
  v[2] /= len;
}

static inline double radToDeg(double angle){
  return (angle * 57.2958);
}

static inline double clamp(double value){
  if(value > 1) return 1;
  if(value < 0) return 0;
  return value;
}


#endif
