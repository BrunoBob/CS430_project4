#include "json_parser.h"
#include "raytracer.h"

//Print all object detected in json file
void printObjects(objectList list){
  while(list != NULL){
    if(list->kind == 0){
      printf("Object of kind : sphere\n");
      printf("Diffuse color : %lf  %lf  %lf\n", list->diffuseColor[0], list->diffuseColor[1], list->diffuseColor[2]);
      printf("Specular color : %lf  %lf  %lf\n", list->specularColor[0], list->specularColor[1], list->specularColor[2]);
      printf("Position : %lf  %lf  %lf\n", list->position[0], list->position[1], list->position[2]);
      printf("Radius : %lf\n", list->sphere.radius);
    }
    else{
      printf("Object of kind : plane\n");
      printf("Diffuse color : %lf  %lf  %lf\n", list->diffuseColor[0], list->diffuseColor[1], list->diffuseColor[2]);
      printf("Specular color : %lf  %lf  %lf\n", list->specularColor[0], list->specularColor[1], list->specularColor[2]);
      printf("Position : %lf  %lf  %lf\n", list->position[0], list->position[1], list->position[2]);
      printf("Normal : %lf  %lf  %lf\n", list->plane.normal[0], list->plane.normal[1], list->plane.normal[2]);
    }
    printf("Refractivity : %lf\n", list->refractivity);
    printf("Reflectivity : %lf\n", list->reflectivity);
    printf("ior : %lf\n", list->ior);
    printf("\n");
    list = list->next;
  }
}

//Print all lights detected in json file
void printLights(lightList list){
  while (list != NULL) {
    if(list->theta == 0){
      printf("\n\n Point light\n");
    }
    else{
      printf("\n\n spotlight\n");
    }
    printf("Light : \nLights color : %lf  %lf  %lf\n", list->color[0], list->color[1], list->color[2]);
    printf("Position : %lf  %lf  %lf\n", list->position[0], list->position[1], list->position[2]);
    printf("Direction : %lf  %lf  %lf\n", list->direction[0], list->direction[1], list->direction[2]);
    printf("Radial-a0 : %lf\n", list->radA0);
    printf("Radial-a0 : %lf\n", list->radA1);
    printf("Radial-a0 : %lf\n", list->radA2);
    if(list->theta != 0){
      printf("Theta : %lf\n", list->theta);
      printf("Angular-a0 : %lf\n", list->angA0);
    }

    printf("\n");
    list = list->next;
  }
}


//Compute if interserction with a plane
double planeIntersection(double* Ro, double* Rd, double* position, double* normal){
  double t = INFINITY;
  normalize(normal); //Just in case
  double denom = dotProduct(normal, Rd);
  if(sqrt(sqr(denom)) > 0.00001){
    t = (-dotProduct(subVector(Ro, position), normal)) / denom;
  }
  return t;
}

//Compute if interserction with a sphere
double sphereIntersection(double* Ro, double* Rd, double* position, double radius){
  double t = INFINITY;

  double* RoSubPosition = subVector(Ro, position);
  double b = 2 * dotProduct(Rd, RoSubPosition);
  double c = dotProduct(RoSubPosition, RoSubPosition) - radius;

  double sqrtDelta = sqrt(sqr(b) - 4 * c);

  t = (-b - sqrtDelta) / 2;

  if(t < 0){
    t = (-b + sqrtDelta) / 2;
  }

  return t;
}


//Write the data in a P6 ppm file
void createScene(char* ppm, unsigned char* data, int width, int height){
  FILE* outputFile = fopen(ppm, "w");

  if (outputFile == NULL) {
    fprintf(stderr, "Error: Could not open file \"%s\"\n", ppm);
    exit(ERROR_WRITING);
  }

  if(fprintf(outputFile, "P6\n#Written by raycaster program made by Bruno TESSIER\n%d %d\n255\n", width, height) < 63){
    fprintf(stderr, "Error: Could not write header in file \"%s\"\n", ppm);
    exit(ERROR_WRITING);
  }
  if(fwrite(data, sizeof(char), width * height * 3, outputFile) != (width * height * 3)){
    fprintf(stderr, "Error: Could not write data in file \"%s\"\n", ppm);
    exit(ERROR_WRITING);
  }
  fclose(outputFile);
}

//Compute angular attenuation of a light
double fAng(double* Vo, double* Vl, double angleMax, double a0){
  if(angleMax == 0){
    return 1; //Not spotlight
  }
  double dot = dotProduct(Vo, Vl);
  double angle = radToDeg(acos(dot));

  if(angle > angleMax){
    return 0;
  }

  //if (dot <0) (dot = dot * -1);
  return (pow(dot, a0));
}

//Compute radial attenuation of a light
double fRad(double dist, double a0, double a1, double a2){
  if(dist == INFINITY){
    return 1;
  }
  return 1/(a2*sqr(dist + a1*dist + a0));
}

//Compute the incident light
double* diffuse(double* objDiffuse, double* lightColor, double* N, double* L){
  double NL = dotProduct(N, L);
  if(NL > 0){
    return scaleVector(multVector(objDiffuse, lightColor), NL);
  }
  return getVector(0,0,0);
}

//compute the specular light
double* specular(double* objSpecular, double* lightColor, double* R, double* V,  double* N, double* L, double shininess){
  double RV = dotProduct(R, V);
  double NL = dotProduct(N, L);
  if(NL > 0 && RV > 0){
    return scaleVector(multVector(objSpecular, lightColor), pow(RV, shininess));
  }
  return getVector(0,0,0);
}

//Chek if interserction of a ray to an object
double shoot(double* Ro, double* Rd, objectList object){
  double t;

  switch (object->kind) {
    case 0:
    t = sphereIntersection(Ro, Rd, object->position, object->sphere.radius);
    break;
    case 1:
    t = planeIntersection(Ro, Rd, object->position, object->plane.normal);
    break;
    default:
    fprintf(stderr, "Error: Object of kind unknow (How is it even possible ?)");
    exit(ERROR_RAYCAST);
  }
  return t;
}

//Compute the direct lightning of an object
double* directShade(double* color, lightList light, objectList object, double* Rdn, double* Rd, double* Vo, double* Ron, double dist){
  double* N = NULL;
  double* L = NULL;
  double* R = NULL;
  double* V = NULL;

  if(object->kind == 1){
    N = object->plane.normal;
  }
  else{
    N = subVector(Ron, object->position);
    normalize(N);
  }
  L = Rdn;
  normalize(L);
  R = subVector(scaleVector(N, dotProduct(N, L) * 2),L);
  normalize(R);
  V = scaleVector(Rd, -1);
  normalize(V);

  double* diffuseColor = diffuse(object->diffuseColor, light->color, N, L);
  double* specularColor = specular(object->specularColor, light->color, R, V, N, L, 20);

  double angAtt = fAng(Vo, light->direction, light->theta, light->angA0);
  double radAtt = fRad(dist, light->radA0, light->radA1, light->radA2);

  color[0] += angAtt * radAtt * (diffuseColor[0] + specularColor[0]);
  color[1] += angAtt * radAtt * (diffuseColor[1] + specularColor[1]);
  color[2] += angAtt * radAtt * (diffuseColor[2] + specularColor[2]);

  return color;
}

//Compute the light
double* shade(lightList light, objectList allObject, objectList object, double* Ro, double* Rd, double bestT, int level){
  double* color = getVector(0,0,0);
  if(object != NULL){ //If object detected

    double* Ron = addVector(scaleVector(Rd, bestT), Ro); //Position of interserction point


    lightList tempLights = light;

    while(tempLights != NULL){ //For all lights
      double* Rdn = subVector(tempLights->position, Ron); //Vector from point to light
      normalize(Rdn);

      double* Vo = subVector(Ron, tempLights->position);
      double dist = sqrt(sqr(Vo[0]) + sqr(Vo[1]) + sqr(Vo[2]));
      normalize(Vo);

      objectList tempList = allObject;
      double t;
      int shadow = 0;

      //Shadow detection
      while(tempList != NULL){ //For all objects
        double*  Ron2 = addVector(Ron, scaleVector(Rdn, EPSILON));

        t = shoot(Ron2, Rdn, tempList);

        if(t > 0 && t < dist){ //If distance of interserction < distance to light then shadow detected from this light
          shadow = 1;
          break;
        }
        tempList = tempList->next;
      }
      if(!shadow){
        color = directShade(color, tempLights, object, Rdn, Rd, Vo, Ron, dist);
      }
      tempLights = tempLights->next;
    }

    //Compute normal vector of the object
    double* N;
    if(object->kind == 1){
      N = object->plane.normal;
    }
    else{
      N = subVector(Ron, object->position);
      normalize(N);
    }

    //Compute reflected ray
    double* reflectedRay = subVector(Rd,scaleVector(N, dotProduct(Rd,N)*2)); // Um = ur - 2(Ur.n)n
    normalize(reflectedRay);
    double t;
    double reflectedT = INFINITY;
    objectList tempList = allObject;
    objectList reflectedObject = NULL;

    while(tempList != NULL){ //For all objects
      if(tempList != object){ //

        t = shoot(Ron, reflectedRay, tempList);

        if(t > 0 && t < reflectedT){ //If object detected
          reflectedT = t;
          reflectedObject = tempList;
        }
      }
      tempList = tempList->next;
    }

    double* reflectedColor = shade(light, allObject, reflectedObject, Ro, reflectedRay, reflectedT, level+1);
    reflectedColor = scaleVector(reflectedColor,object->reflectivity);

    //reflected light = reflectivity * shade( rreflected ray);

    //Compute refracted ray

    //refracted light = refractivity * shade (refracted ray);
    color = scaleVector(color, (1 - object->reflectivity));
    color = addVector(color, reflectedColor);
  }
  return color;
}

int main(int argc, char *argv[]){
  if(argc < 5){
    fprintf(stderr, "Error: Expected ./raycaster width height input.json output.ppm");
    exit(ERROR_RAYCAST);
  }

  double centerX = 0;
  double centerY = 0;

  int width = atoi(argv[1]);
  int height = atoi(argv[2]);

  double camWidth, camHeight;

  components comp = NULL;
  comp = parseFile(argv[3], &camWidth, &camHeight);
  objectList list = comp->objects;
  lightList lights = comp->lights;
  printf("%d\n", list->kind);

  double pixWidth = camWidth / width;
  double pixHeight = camHeight / height;

  printf("\nScene : width = %d\theight = %d\n", width, height);
  printf("\nCamera : width = %lf\theight = %lf\n\n", camWidth, camHeight);
  printObjects(list);
  printLights(lights);
  unsigned char* data = (unsigned char*)malloc(width * height * 3 * sizeof(unsigned char));

  int x,y;

  for(y = 0; y < height ; y++){
    for(x = 0; x < width ; x++){
      double* Ro = getVector(0, 0, 0); //Origin of camera
      double Rx = centerX - (camWidth/2) + pixWidth * (x+0.5);
      double Ry = centerY - (camHeight/2) + pixHeight * (y+0.5);
      double* Rd = getVector(Rx, Ry, 1); //vector from camera to pixel

      normalize(Rd);

      double bestT = INFINITY;
      double t = 0;
      objectList closestObject = NULL;
      objectList tempList = list;


      //Closest object detection
      while (tempList != NULL) {

        t = shoot(Ro, Rd, tempList);

        if(t > 0 && t < bestT){ //Select the closest object
          bestT = t;
          closestObject = tempList;
        }
        tempList = tempList->next;
      }

      double* color = getVector(0,0,0);
      lightList tempLights = lights;

      //Shading
      color = shade(tempLights, list, closestObject, Ro, Rd, bestT, 0);

      data[ 3 * (x + width * (height - 1 - y))] = clamp(color[0]) * 255;
      data[ 3 * (x + width * (height - 1 - y)) + 1] = clamp(color[1]) * 255;
      data[ 3 * (x + width * (height - 1 - y)) + 2] = clamp(color[2]) * 255;
    }
  }

  createScene(argv[4], data, width, height); //Write the image
  free(data);

  return 0;
}
