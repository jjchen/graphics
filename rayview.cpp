// Source file for the scene file viewer



////////////////////////////////////////////////////////////
// INCLUDE FILES
////////////////////////////////////////////////////////////

#include "R3/R3.h"
#include "R3Scene.h"
#include "cos426_opengl.h"
#include <sys/time.h>
#include <map>
#include <fstream>
#include <iostream>
#include "client.h"
#include "server.h"

void Update();
double toRads(double degrees);

////////////////////////////////////////////////////////////
// GLOBAL VARIABLES
////////////////////////////////////////////////////////////

// Program arguments

static char *input_scene_name = NULL;

// Display variables

static R3Scene *scene = NULL;
static R3Camera camera;
static int show_faces = 1;
static int show_bboxes = 0;
static int show_lights = 0;
static int show_camera = 0;
static int save_image = 0;
static int quit = 0;

// angle of rotation for the camera direction
float angle = 0.0f;

// actual vector representing the camera's direction
float lx=0.0f,lz=-1.0f, ly = 0.0f;

// XZ position of the camera
float x=0.0f, z=5.0f, y = 1.75f;

// the key states. These variables will be zero
//when no key is being presses
float deltaAngle = 0.0f;
float deltaMove = 0;
int xOrigin = -1;

// GLUT variables 

int GLUTwindow = 0;
int subWindow2 = 0;
static int GLUTwindow_height = 512;
static int GLUTwindow_width = 512;
static int GLUTmouse[2] = { 0, 0 };
static int GLUTbutton[3] = { 0, 0, 0 };
static int GLUTmodifiers = 0;

//gameplay variables
// Player car
static double playerCarXPos = 0.0;
static double playerCarYPos = 0.0;
static double playerCarZPos = 0.0;

// Other car
static double otherCarXPos = 0.0;
static double otherCarYPos = 0.0;
static double otherCarZPos = 0.0;
static double otherCarTheta = 0.0;

// Keys
static bool upPressActive = false;
static bool downPressActive = false;
static bool leftPressActive = false;
static bool rightPressActive = false;

// More networking variables
map<char*, char*> config_map;
static bool connected = false;
static bool is_client = false;
static char* ip_address;
static int port;
static bool use_networking = false;
static int socket_desc;

enum {
  THIRD_PERSON_VIEW,
  DRIVER_SEAT_VIEW
};

static int currentView = THIRD_PERSON_VIEW;
static double camYPos = 25;
static double camZDistance = camYPos;

//gameplay physics variables
static double carAngle = 0.0;
static double carSpeed = 0.0;
static double carAcceleration = 0.0;
double MAX_SPEED = 60;
static double ACCELERATION_DUE_TO_GRAVITY = 9.8;
static double GROUND_FRICTION_COEFFICIENT = 0.7;
static double FRICTION_ACCELERATION_MAGNITUDE = GROUND_FRICTION_COEFFICIENT * ACCELERATION_DUE_TO_GRAVITY;
static double FORWARD_AND_REVERSE_ACCELRATION_MAGNITUDE = 12.0; 
static double ANGLE_TURN_RATIO = 0.1;
char time_str[50];
char lap_str[50];
int lap = 1;

// GLUT command list

enum {
  DISPLAY_FACE_TOGGLE_COMMAND,
  DISPLAY_EDGE_TOGGLE_COMMAND,
  DISPLAY_BBOXES_TOGGLE_COMMAND,
  DISPLAY_LIGHTS_TOGGLE_COMMAND,
  DISPLAY_CAMERA_TOGGLE_COMMAND,
  SAVE_IMAGE_COMMAND,
  QUIT_COMMAND,
};

/////////////////////////////////////////////////////////////
// Miscellaneous Functions
/////////////////////////////////////////////////////////////

/* Returns the seconds since start of execution. */
static double GetTime(void)
{
#ifdef _WIN32
  // Return number of seconds since start of execution
  static int first = 1;
  static LARGE_INTEGER timefreq;
  static LARGE_INTEGER start_timevalue;

  // Check if this is the first time
  if (first) {
    // Initialize first time
    QueryPerformanceFrequency(&timefreq);
    QueryPerformanceCounter(&start_timevalue);
    first = 0;
    return 0;
  }
  else {
    // Return time since start
    LARGE_INTEGER current_timevalue;
    QueryPerformanceCounter(&current_timevalue);
    return ((double) current_timevalue.QuadPart - 
            (double) start_timevalue.QuadPart) / 
            (double) timefreq.QuadPart;
  }
#else
  // Return number of seconds since start of execution
  static int first = 1;
  static struct timeval start_timevalue;

  // Check if this is the first time
  if (first) {
    // Initialize first time
    gettimeofday(&start_timevalue, NULL);
    first = 0;
    return 0;
  }
  else {
    // Return time since start
    struct timeval current_timevalue;
    gettimeofday(&current_timevalue, NULL);
    int secs = current_timevalue.tv_sec - start_timevalue.tv_sec;
    int usecs = current_timevalue.tv_usec - start_timevalue.tv_usec;
    return (double) (secs + 1.0E-6F * usecs);
  }
#endif
}

////////////////////////////////////////////////////////////
// SCENE DRAWING CODE
////////////////////////////////////////////////////////////

void DrawShape(R3Shape *shape)
{
  // Check shape type
  if (shape->type == R3_BOX_SHAPE) shape->box->Draw();
  else if (shape->type == R3_SPHERE_SHAPE) shape->sphere->Draw();
  else if (shape->type == R3_CYLINDER_SHAPE) shape->cylinder->Draw();
  else if (shape->type == R3_CONE_SHAPE) shape->cone->Draw();
  else if (shape->type == R3_MESH_SHAPE) shape->mesh->Draw();
  else if (shape->type == R3_SEGMENT_SHAPE) shape->segment->Draw();
  else fprintf(stderr, "Unrecognized shape type: %d\n", shape->type);
}



void LoadMatrix(R3Matrix *matrix)
{
  // Multiply matrix by top of stack
  // Take transpose of matrix because OpenGL represents vectors with 
  // column-vectors and R3 represents them with row-vectors
  R3Matrix m = matrix->Transpose();
  glMultMatrixd((double *) &m);
}

void LoadMaterial(R3Material *material) 
{
  GLfloat c[4];

  // Check if same as current
  static R3Material *current_material = NULL;
  if (material == current_material) return;
  current_material = material;

  // Compute "opacity"
  double opacity = 1 - material->kt.Luminance();

  // Load ambient
  c[0] = material->ka[0];
  c[1] = material->ka[1];
  c[2] = material->ka[2];
  c[3] = opacity;
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, c);

  // Load diffuse
  c[0] = material->kd[0];
  c[1] = material->kd[1];
  c[2] = material->kd[2];
  c[3] = opacity;
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, c);

  // Load specular
  c[0] = material->ks[0];
  c[1] = material->ks[1];
  c[2] = material->ks[2];
  c[3] = opacity;
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, c);

  // Load emission
  c[0] = material->emission.Red();
  c[1] = material->emission.Green();
  c[2] = material->emission.Blue();
  c[3] = opacity;
  glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, c);

  // Load shininess
  c[0] = material->shininess;
  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, c[0]);

  // Load texture
  if (material->texture) {
    if (material->texture_index <= 0) {
      // Create texture in OpenGL
      GLuint texture_index;
      glGenTextures(1, &texture_index);
      material->texture_index = (int) texture_index;
      glBindTexture(GL_TEXTURE_2D, material->texture_index); 
      R2Image *image = material->texture;
      int npixels = image->NPixels();
      R2Pixel *pixels = image->Pixels();
      GLfloat *buffer = new GLfloat [ 4 * npixels ];
      R2Pixel *pixelsp = pixels;
      GLfloat *bufferp = buffer;
      for (int j = 0; j < npixels; j++) { 
        *(bufferp++) = pixelsp->Red();
        *(bufferp++) = pixelsp->Green();
        *(bufferp++) = pixelsp->Blue();
        *(bufferp++) = pixelsp->Alpha();
        pixelsp++;
      }
      glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
      glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
      glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
      glTexImage2D(GL_TEXTURE_2D, 0, 4, image->Width(), image->Height(), 0, GL_RGBA, GL_FLOAT, buffer);
      delete [] buffer;
    }

    // Select texture
    glBindTexture(GL_TEXTURE_2D, material->texture_index); 
    glEnable(GL_TEXTURE_2D);
  }
  else {
    glDisable(GL_TEXTURE_2D);
  }

  // Enable blending for transparent surfaces
  if (opacity < 1) {
    glDepthMask(false);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
  }
  else {
    glDisable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ZERO);
    glDepthMask(true);
  }
}

double toRads(double degrees)
{
  return (3.14159 / 180.0) * degrees;
}
//updates objects in world, such as car
//called at beginning of rendering cycle -- change elseifs to ifs when
//physics in place
void Update()
{
  // Get current time (in seconds) since start of execution
  double current_time = GetTime();
  static double previous_time = 0;

  // program just started up?
  if (previous_time == 0) previous_time = current_time;

  // time passed since starting
  double delta_time = current_time - previous_time;
  
  //update car position
  playerCarXPos += carSpeed * sin(toRads(carAngle)) * delta_time; //may need to change to -=
  playerCarZPos += carSpeed * cos(toRads(carAngle)) * delta_time;
  
  //update car speed
  double newSpeed = carSpeed + carAcceleration * delta_time;
  
  //if car acceleration magnitude is equal to ground acceleration magnitude
  //then check if velocity would change signs after this update, in which case
  //set the velocity to 0 (friction can't make you change directions by taking you "past" 0)
  if (abs(carAcceleration) == FRICTION_ACCELERATION_MAGNITUDE)
  {
    bool previousSpeedPositive = (carSpeed >= 0);
    bool newSpeedPositive = (newSpeed) >= 0;
    if (previousSpeedPositive != newSpeedPositive)
    {
      carSpeed = 0.0;
    }
    else
    {
      carSpeed = newSpeed;
    }
  }
  else
  {
    carSpeed = newSpeed;
  }
  
  //clamp car speed
  if (carSpeed > MAX_SPEED) carSpeed = MAX_SPEED;
  if (carSpeed < -1.0 * MAX_SPEED) carSpeed = -1.0 * MAX_SPEED;
  
  //update car acceleration (and reset speed to 0 if press up/down to change directions)
  if (upPressActive)
  {
    carAcceleration = FORWARD_AND_REVERSE_ACCELRATION_MAGNITUDE;
    if (carSpeed < 0) carSpeed = 0;
  }
  else if (downPressActive)
  {
    carAcceleration = -1.0 * FORWARD_AND_REVERSE_ACCELRATION_MAGNITUDE;
    if (carSpeed > 0) carSpeed = 0;
  }
  else
  {
    carAcceleration = (carSpeed >= 0.0) ? -1.0 * FRICTION_ACCELERATION_MAGNITUDE : FRICTION_ACCELERATION_MAGNITUDE;
  }
  
  if (leftPressActive)
  {
    carAngle += ANGLE_TURN_RATIO * carSpeed;
    carAngle = fmod(carAngle, 360);
  }
  else if (rightPressActive)
  {
    carAngle -= ANGLE_TURN_RATIO * carSpeed;
    carAngle = fmod(carAngle, 360);
  }
  
  // update previous time
  previous_time = current_time;
}

void LoadCamera(R3Camera *camera)
{
  // Set projection transformation
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glViewport(0, 0, GLUTwindow_width, GLUTwindow_height);
  //gluPerspective(2*180.0*camera->yfov/M_PI, (GLdouble) GLUTwindow_width /(GLdouble) GLUTwindow_height, 0.01, 10000);
  gluPerspective(2*180.0*camera->yfov/M_PI, (GLdouble) GLUTwindow_width /(GLdouble) GLUTwindow_height, .01, 10000);

  // Set camera transformation
  if (currentView == THIRD_PERSON_VIEW)
  { 
   //set camera eye by assuming car at the origin then translating
   camera->eye = R3Point(0, camYPos, -1.0 * camZDistance);
   camera->eye.Rotate(R3Vector(0.0, 1.0, 0.0), carAngle * (3.14159 / 180.0));
   camera->eye += R3Point(playerCarXPos, playerCarYPos, playerCarZPos);
   
   camera->towards = R3Vector(playerCarXPos - camera->eye[0], playerCarYPos - camera->eye[1], 
    playerCarZPos - camera->eye[2]);
   camera->towards.Normalize();
   camera->up = R3Vector(0.0, 1.0, 0.0);
   camera->right = camera->towards % camera->up;
   camera->right.Normalize();
   camera->up = camera->right % camera->towards;
   camera->up.Normalize();
  }
  //when implement first person, add code here
  else
  {
    
  }
  
  R3Vector t = -(camera->towards);
  R3Vector& u = camera->up;
  R3Vector& r = camera->right;
  
  GLdouble camera_matrix[16] = { r[0], u[0], t[0], 0, r[1], u[1], t[1], 0, r[2], u[2], t[2], 0, 0, 0, 0, 1 };
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glMultMatrixd(camera_matrix);
  glTranslated(-(camera->eye[0]), -(camera->eye[1]), -(camera->eye[2]));
  //glRotatef(-1.0 * carAngle * 1000.0 / (R3Distance(camera->eye, R3Point(playerCarXPos, playerCarYPos, playerCarZPos))) , 0.0, 1.0, 0.0);
}

void LoadLights(R3Scene *scene)
{
  GLfloat buffer[4];

  // Load ambient light
  static GLfloat ambient[4];
  ambient[0] = scene->ambient[0];
  ambient[1] = scene->ambient[1];
  ambient[2] = scene->ambient[2];
  ambient[3] = 1;
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);

  // Load scene lights
  for (int i = 0; i < (int) scene->lights.size(); i++) {
    R3Light *light = scene->lights[i];
    int index = GL_LIGHT0 + i;

    // Temporarily disable light
    glDisable(index);

    // Load color
    buffer[0] = light->color[0];
    buffer[1] = light->color[1];
    buffer[2] = light->color[2];
    buffer[3] = 1.0;
    glLightfv(index, GL_DIFFUSE, buffer);
    glLightfv(index, GL_SPECULAR, buffer);

    // Load attenuation with distance
    buffer[0] = light->constant_attenuation;
    buffer[1] = light->linear_attenuation;
    buffer[2] = light->quadratic_attenuation;
    glLightf(index, GL_CONSTANT_ATTENUATION, buffer[0]);
    glLightf(index, GL_LINEAR_ATTENUATION, buffer[1]);
    glLightf(index, GL_QUADRATIC_ATTENUATION, buffer[2]);

    // Load spot light behavior
    buffer[0] = 180.0 * light->angle_cutoff / M_PI;
    buffer[1] = light->angle_attenuation;
    glLightf(index, GL_SPOT_CUTOFF, buffer[0]);
    glLightf(index, GL_SPOT_EXPONENT, buffer[1]);

    // Load positions/directions
    if (light->type == R3_DIRECTIONAL_LIGHT) {
      // Load direction
      buffer[0] = -(light->direction.X());
      buffer[1] = -(light->direction.Y());
      buffer[2] = -(light->direction.Z());
      buffer[3] = 0.0;
      glLightfv(index, GL_POSITION, buffer);
    }
    else if (light->type == R3_POINT_LIGHT) {
      // Load position
      buffer[0] = light->position.X();
      buffer[1] = light->position.Y();
      buffer[2] = light->position.Z();
      buffer[3] = 1.0;
      glLightfv(index, GL_POSITION, buffer);
    }
    else if (light->type == R3_SPOT_LIGHT) {
      // Load position
      buffer[0] = light->position.X();
      buffer[1] = light->position.Y();
      buffer[2] = light->position.Z();
      buffer[3] = 1.0;
      glLightfv(index, GL_POSITION, buffer);

      // Load direction
      buffer[0] = light->direction.X();
      buffer[1] = light->direction.Y();
      buffer[2] = light->direction.Z();
      buffer[3] = 1.0;
      glLightfv(index, GL_SPOT_DIRECTION, buffer);
    }
    else if (light->type == R3_AREA_LIGHT) {
      // Load position
      buffer[0] = light->position.X();
      buffer[1] = light->position.Y();
      buffer[2] = light->position.Z();
      buffer[3] = 1.0;
      glLightfv(index, GL_POSITION, buffer);

      // Load direction
      buffer[0] = light->direction.X();
      buffer[1] = light->direction.Y();
      buffer[2] = light->direction.Z();
      buffer[3] = 1.0;
      glLightfv(index, GL_SPOT_DIRECTION, buffer);
    }
     else {
      fprintf(stderr, "Unrecognized light type: %d\n", light->type);
      return;
    }

    // Enable light
    glEnable(index);
  }
}

void DrawNode(R3Scene *scene, R3Node *node, bool isOverview)
{
  // Push transformation onto stack
  glPushMatrix();
  LoadMatrix(&node->transformation);
  
  //if car, update position and translate to it
  if (node->isPlayerCarMesh)
  {
    glTranslatef(playerCarXPos, playerCarYPos, playerCarZPos);
    glRotatef(carAngle, 0.0, 1.0, 0.0);
  }
  

  // Load material
  if (node->material) LoadMaterial(node->material);

  // Draw shape
  if (node->shape && !(node->isPlayerCarMesh && isOverview)) DrawShape(node->shape);
  else {

  }
  //if car, reload identity matrix
  if (node->isPlayerCarMesh) 
  {
    R3Matrix r = R3Matrix(1.0, 0.0, 0.0, 0.0, 
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 
    0.0, 0.0, 0.0, 1.0);
    LoadMatrix(&r);
   }
  
  // Draw children nodes
  for (int i = 0; i < (int) node->children.size(); i++) 
    DrawNode(scene, node->children[i], isOverview);

  // Restore previous transformation
  glPopMatrix();

  // Show bounding box
  if (show_bboxes) {
    GLboolean lighting = glIsEnabled(GL_LIGHTING);
    glDisable(GL_LIGHTING);
    node->bbox.Outline();
    if (lighting) glEnable(GL_LIGHTING);
  }
}

void DrawLights(R3Scene *scene)
{
  // Check if should draw lights
  if (!show_lights) return;

  // Setup
  GLboolean lighting = glIsEnabled(GL_LIGHTING);
  glDisable(GL_LIGHTING);

  // Draw all lights
  double radius = scene->bbox.DiagonalRadius();
  for (int i = 0; i < scene->NLights(); i++) {
    R3Light *light = scene->Light(i);
    glColor3d(light->color[0], light->color[1], light->color[2]);
    if (light->type == R3_DIRECTIONAL_LIGHT) {
      // Draw direction vector
      glLineWidth(5);
      glBegin(GL_LINES);
      R3Point centroid = scene->bbox.Centroid();
      R3Vector vector = radius * light->direction;
      glVertex3d(centroid[0] - vector[0], centroid[1] - vector[1], centroid[2] - vector[2]);
      glVertex3d(centroid[0] - 1.25*vector[0], centroid[1] - 1.25*vector[1], centroid[2] - 1.25*vector[2]);
      glEnd();
      glLineWidth(1);
    }
    else if (light->type == R3_POINT_LIGHT) {
      // Draw sphere at point light position
      R3Sphere(light->position, 0.1 * radius).Draw();
    }
    else if (light->type == R3_SPOT_LIGHT) {
      // Draw sphere at point light position and line indicating direction
      R3Sphere(light->position, 0.1 * radius).Draw();
  
      // Draw direction vector
      glLineWidth(5);
      glBegin(GL_LINES);
      R3Vector vector = radius * light->direction;
      glVertex3d(light->position[0], light->position[1], light->position[2]);
      glVertex3d(light->position[0] + 0.25*vector[0], light->position[1] + 0.25*vector[1], light->position[2] + 0.25*vector[2]);
      glEnd();
      glLineWidth(1);
    }
    else if (light->type == R3_AREA_LIGHT) {
      // Draw circular area
      R3Vector v1, v2;
      double r = light->radius;
      R3Point p = light->position;
      int dim = light->direction.MinDimension();
      if (dim == 0) { v1 = light->direction % R3posx_vector; v1.Normalize(); v2 = light->direction % v1; }
      else if (dim == 1) { v1 = light->direction % R3posy_vector; v1.Normalize(); v2 = light->direction % v1; }
      else { v1 = light->direction % R3posz_vector; v1.Normalize(); v2 = light->direction % v1; }
      glBegin(GL_POLYGON);
      glVertex3d(p[0] +  1.00*r*v1[0] +  0.00*r*v2[0], p[1] +  1.00*r*v1[1] +  0.00*r*v2[1], p[2] +  1.00*r*v1[2] +  0.00*r*v2[2]);
      glVertex3d(p[0] +  0.71*r*v1[0] +  0.71*r*v2[0], p[1] +  0.71*r*v1[1] +  0.71*r*v2[1], p[2] +  0.71*r*v1[2] +  0.71*r*v2[2]);
      glVertex3d(p[0] +  0.00*r*v1[0] +  1.00*r*v2[0], p[1] +  0.00*r*v1[1] +  1.00*r*v2[1], p[2] +  0.00*r*v1[2] +  1.00*r*v2[2]);
      glVertex3d(p[0] + -0.71*r*v1[0] +  0.71*r*v2[0], p[1] + -0.71*r*v1[1] +  0.71*r*v2[1], p[2] + -0.71*r*v1[2] +  0.71*r*v2[2]);
      glVertex3d(p[0] + -1.00*r*v1[0] +  0.00*r*v2[0], p[1] + -1.00*r*v1[1] +  0.00*r*v2[1], p[2] + -1.00*r*v1[2] +  0.00*r*v2[2]);
      glVertex3d(p[0] + -0.71*r*v1[0] + -0.71*r*v2[0], p[1] + -0.71*r*v1[1] + -0.71*r*v2[1], p[2] + -0.71*r*v1[2] + -0.71*r*v2[2]);
      glVertex3d(p[0] +  0.00*r*v1[0] + -1.00*r*v2[0], p[1] +  0.00*r*v1[1] + -1.00*r*v2[1], p[2] +  0.00*r*v1[2] + -1.00*r*v2[2]);
      glVertex3d(p[0] +  0.71*r*v1[0] + -0.71*r*v2[0], p[1] +  0.71*r*v1[1] + -0.71*r*v2[1], p[2] +  0.71*r*v1[2] + -0.71*r*v2[2]);
      glEnd();
    }
    else {
      fprintf(stderr, "Unrecognized light type: %d\n", light->type);
      return;
    }
  }

  // Clean up
  if (lighting) glEnable(GL_LIGHTING);
}

void DrawCamera(R3Scene *scene)
{
  // Check if should draw lights
  if (!show_camera) return;

  // Setup
  GLboolean lighting = glIsEnabled(GL_LIGHTING);
  glDisable(GL_LIGHTING);
  glColor3d(1.0, 1.0, 1.0);
  glLineWidth(5);

  // Draw view frustum
  R3Camera& c = scene->camera;
  double radius = scene->bbox.DiagonalRadius();
  R3Point org = c.eye + c.towards * radius;
  R3Vector dx = c.right * radius * tan(c.xfov);
  R3Vector dy = c.up * radius * tan(c.yfov);
  R3Point ur = org + dx + dy;
  R3Point lr = org + dx - dy;
  R3Point ul = org - dx + dy;
  R3Point ll = org - dx - dy;
  glBegin(GL_LINE_LOOP);
  glVertex3d(ur[0], ur[1], ur[2]);
  glVertex3d(ul[0], ul[1], ul[2]);
  glVertex3d(ll[0], ll[1], ll[2]);
  glVertex3d(lr[0], lr[1], lr[2]);
  glVertex3d(ur[0], ur[1], ur[2]);
  glVertex3d(c.eye[0], c.eye[1], c.eye[2]);
  glVertex3d(lr[0], lr[1], lr[2]);
  glVertex3d(ll[0], ll[1], ll[2]);
  glVertex3d(c.eye[0], c.eye[1], c.eye[2]);
  glVertex3d(ul[0], ul[1], ul[2]);
  glEnd();

  // Clean up
  glLineWidth(1);
  if (lighting) glEnable(GL_LIGHTING);
}

void DrawScene(R3Scene *scene, bool isOverview) 
{
  // Draw nodes recursively
  DrawNode(scene, scene->root, isOverview);
}

////////////////////////////////////////////////////////////
// GLUT USER INTERFACE CODE
////////////////////////////////////////////////////////////

void GLUTMainLoop(void)
{
  // Run main loop -- never returns 
  glutMainLoop();
}

void GLUTDrawText(const R3Point& p, const char *s)
{
  // Draw text string s and position p
 
  glRasterPos3d(p[0], p[1], p[2]);
#ifndef __CYGWIN__
  while (*s) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *(s++));
#else
  while (*s) glutBitmapCharacter((void*)7, *(s++));
#endif
}


void GLUTPrint(double x, double y, char * text)
{
  glRasterPos2f(x, y);
  #ifndef __CYGWIN__
    while (*text) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *(text++));
  #else
    while (*text) glutBitmapCharacter((void*)7, *(text++));
  #endif  
}

void GLUTStop(void)
{
  // Destroy window 
  glutDestroyWindow(GLUTwindow);

  // Delete scene
  delete scene;

  // Exit
  exit(0);
}

void GLUTResize(int w, int h)
{
  // Resize window
  glViewport(0, 0, w, h);

  // Resize camera vertical field of view to match aspect ratio of viewport
  camera.yfov = atan(tan(camera.xfov) * (double) h/ (double) w); 

  // Remember window size 
  GLUTwindow_width = w;
  GLUTwindow_height = h;

  // Redraw
  glutPostRedisplay();
}


void drawHUD(void)
{
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0.0, 500, 500, 0.0, -1.0, 10.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glDisable(GL_LIGHTING); 

    glClear(GL_DEPTH_BUFFER_BIT);
    glColor3d(1.0f, 1.0f, 1.0f);
    
    double curtime = GetTime();
    static double prevtime = 0;
    if (prevtime == 0) prevtime = curtime;
    sprintf(time_str, "Time: %4.1f", curtime);
    prevtime = curtime;

    sprintf(lap_str, "Lap: %d of 3", lap);

    GLUTPrint(400,75, time_str);
    GLUTPrint(50,400,"Position: 2nd out of 10");
    GLUTPrint(50,450,"Speed: 60mph");
    GLUTPrint(385,360,"Track");
    GLUTPrint(400,50, lap_str);

    glBegin(GL_QUADS);
        glVertex2f(380, 380); // vertex 1
        glVertex2f(380, 475); // vertex 2
        glVertex2f(475, 475); // vertex 3
        glVertex2f(475, 380); // vertex 4
    glEnd();
    glEnable(GL_LIGHTING); 

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);  
}

void drawMiniMapView(void) {
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   glViewport(2*GLUTwindow_width/3, 0, GLUTwindow_width/3 , GLUTwindow_height/3);
   gluPerspective(2*180.0*camera.yfov/M_PI, (GLdouble) GLUTwindow_width /(GLdouble) GLUTwindow_height, .01, 10000);
   glMatrixMode(GL_MODELVIEW);
   glScissor((2*GLUTwindow_width/3), 0, GLUTwindow_width/3 , GLUTwindow_height/3);
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
   glLoadIdentity();
   R3Vector t = -(camera.towards);

   //gluLookAt(0,100,0, 0, 99, 0, t[0], 0, t[1]);
   gluLookAt(0,200,0, 0, 99, 0, 0, 0, 1);
   LoadLights(scene);
   glEnable(GL_LIGHTING);
   DrawScene(scene, false);
}

void drawRearViewMirror(void) {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glViewport(10, 4*GLUTwindow_height/5-10, GLUTwindow_width/5 , GLUTwindow_height/5);
    gluPerspective(2*180.0*camera.yfov/M_PI, (GLdouble) GLUTwindow_width /(GLdouble) GLUTwindow_height, .01, 10000);
    glMatrixMode(GL_MODELVIEW);
    glScissor(10, 4*GLUTwindow_height/5-10, GLUTwindow_width/5 , GLUTwindow_height/5);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    R3Vector t = R3Vector(camera.towards.X(), 0, camera.towards.Z());
    R3Vector r = (camera.right);
    R3Vector u = r % t;
    
    GLdouble camera_matrix[16] = { r[0], u[0], t[0], 0, r[1], u[1], t[1], 0, r[2], u[2], t[2], 0, 0, 0, 0, 1 };
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glMultMatrixd(camera_matrix);
    glTranslated(-(camera.eye[0]), playerCarYPos-10, -(camera.eye[2]));

    //gluLookAt(0,200,0, 0, 99, 0, 0, 0, 1);
    LoadLights(scene);
    glEnable(GL_LIGHTING);
    DrawScene(scene, false);
}

void LoadHeadLight(void) {
  GLfloat lightbuffer[4];

  glDisable(GL_LIGHT5);
  GLfloat headlight_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
  GLfloat headlight_specular[] = { 1.0, 1.0, 1.0, 1.0 };

  glLightfv(GL_LIGHT5, GL_DIFFUSE, headlight_diffuse);
  glLightfv(GL_LIGHT5, GL_SPECULAR, headlight_specular);

  glLightf(GL_LIGHT5, GL_CONSTANT_ATTENUATION, 1);
  glLightf(GL_LIGHT5, GL_LINEAR_ATTENUATION, 1);
  glLightf(GL_LIGHT5, GL_QUADRATIC_ATTENUATION, 1);

  // Load spot light behavior

  glLightf(GL_LIGHT5, GL_SPOT_CUTOFF, 1);
  glLightf(GL_LIGHT5, GL_SPOT_EXPONENT, 1);

  lightbuffer[0] = 0;
  lightbuffer[1] = 0;
  lightbuffer[2] = 0;
  lightbuffer[3] = 1.0;

  glLightfv(GL_LIGHT5, GL_POSITION, lightbuffer);

  lightbuffer[0] = 0;
  lightbuffer[1] = 0;
  lightbuffer[2] = 0;
  lightbuffer[3] = 1.0;  
  glLightfv(GL_LIGHT5, GL_SPOT_DIRECTION, lightbuffer);

  glEnable(GL_LIGHT5);
}

void GLUTRedrawMain(void)
{
  Update();

  glutSetWindow(GLUTwindow);
  glEnable(GL_SCISSOR_TEST);

  // Initialize OpenGL drawing modes
  glEnable(GL_LIGHTING);
  glDisable(GL_BLEND);
  glBlendFunc(GL_ONE, GL_ZERO);
  glDepthMask(true);

  // Clear window 
  R3Rgb background = scene->background;
  glClearColor(background[0], background[1], background[2], background[3]);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Load camera
  glScissor(0, 0, GLUTwindow_width , GLUTwindow_height);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  LoadCamera(&camera);

  // Load scene lights
  LoadLights(scene);

  LoadHeadLight();

  // Draw scene surfaces
  if (show_faces) {
    glEnable(GL_LIGHTING);
    DrawScene(scene, false);
  }
  drawHUD();

  drawMiniMapView();
  drawRearViewMirror();
  glDisable(GL_SCISSOR_TEST);

  // Swap buffers 
  glFlush();
  glutSwapBuffers();
  
  // Network Stuff
  if (connected) {
      fprintf(stderr, "about to listen/send\n");

      if (is_client) {
          char* data = "opinions!\n";
          if (client_write(data, socket_desc) == 0) {
              fprintf(stderr, "Successful client send (probably)\n");
          }
          else {
              fprintf(stderr, "Couldn't send to server!\n");
          }
      }
      else {
         char* data_received = server_receive(socket_desc);
         if (data_received != NULL) {
             fprintf(stderr, "Got data: %s\n", data_received);
         }
         else {
             fprintf(stderr, "No data... ;(\n");
         }
      }
  }
}    

void GLUTMotion(int x, int y)
{
  // Invert y coordinate
  y = GLUTwindow_height - y;
  
  // Compute mouse movement
  int dx = x - GLUTmouse[0];
  int dy = y - GLUTmouse[1];
  
  // Process mouse motion event
  if ((dx != 0) || (dy != 0)) {
    R3Point scene_center = scene->bbox.Centroid();
    if ((GLUTbutton[0] && (GLUTmodifiers & GLUT_ACTIVE_SHIFT)) || GLUTbutton[1]) {
      // Scale world 
      double factor = (double) dx / (double) GLUTwindow_width;
      factor += (double) dy / (double) GLUTwindow_height;
      factor = exp(2.0 * factor);
      factor = (factor - 1.0) / factor;
      R3Vector translation = (scene_center - camera.eye) * factor;
      camera.eye += translation;
      glutPostRedisplay();
    }
    else if (GLUTbutton[0] && (GLUTmodifiers & GLUT_ACTIVE_CTRL)) {
      // Translate world
      double length = R3Distance(scene_center, camera.eye) * tan(camera.yfov);
      double vx = length * (double) dx / (double) GLUTwindow_width;
      double vy = length * (double) dy / (double) GLUTwindow_height;
      R3Vector translation = -((camera.right * vx) + (camera.up * vy));
      camera.eye += translation;
      glutPostRedisplay();
    }
    else if (GLUTbutton[0]) {
      // Rotate world
      double vx = (double) dx / (double) GLUTwindow_width;
      double vy = (double) dy / (double) GLUTwindow_height;
      double theta = 4.0 * (fabs(vx) + fabs(vy));
      R3Vector vector = (camera.right * vx) + (camera.up * vy);
      R3Vector rotation_axis = camera.towards % vector;
      rotation_axis.Normalize();
      camera.eye.Rotate(R3Line(scene_center, rotation_axis), theta);
      camera.towards.Rotate(rotation_axis, theta);
      camera.up.Rotate(rotation_axis, theta);
      camera.right = camera.towards % camera.up;
      camera.up = camera.right % camera.towards;
      camera.towards.Normalize();
      camera.up.Normalize();
      camera.right.Normalize();
      glutPostRedisplay();
    }
  }

  // Remember mouse position 
  GLUTmouse[0] = x;
  GLUTmouse[1] = y;
}

void GLUTSpecial(int key, int x, int y)
{
  // Invert y coordinate
  y = GLUTwindow_height - y;

  // Process keyboard button event 
  switch (key) {
  case GLUT_KEY_F1:
    save_image = 1;
    break;
  case GLUT_KEY_UP:
    upPressActive = true;
    downPressActive = false;
  break;
  case GLUT_KEY_DOWN:
    downPressActive = true;
    upPressActive = false;
    break;
  case GLUT_KEY_LEFT:
    leftPressActive = true;
    rightPressActive = false;
    break;
  case GLUT_KEY_RIGHT:
    rightPressActive = true;
    leftPressActive = false;
  break;
  }

  // Remember mouse position 
  GLUTmouse[0] = x;
  GLUTmouse[1] = y;

  // Remember modifiers 
  GLUTmodifiers = glutGetModifiers();

  // Redraw
  glutPostRedisplay();
}

void GLUTSpecialUp(int key, int x, int y)
{
  // Invert y coordinate
  y = GLUTwindow_height - y;

  // Process keyboard button event 
  switch (key) {
  case GLUT_KEY_UP:
    upPressActive = false;
  break;
  case GLUT_KEY_DOWN:
    downPressActive = false;
    break;
  case GLUT_KEY_LEFT:
    leftPressActive = false;
    break;
  case GLUT_KEY_RIGHT:
    rightPressActive = false;
  break;
  }

  // Remember mouse position 
  GLUTmouse[0] = x;
  GLUTmouse[1] = y;

  // Remember modifiers 
  GLUTmodifiers = glutGetModifiers();

  // Redraw
  glutPostRedisplay();
}

void GLUTIdle(void)
{
  // Set current window
  if ( glutGetWindow() != GLUTwindow ) 
    glutSetWindow(GLUTwindow);  

  // Redraw
  glutPostRedisplay();
}

void GLUTInit(int *argc, char **argv)
{
  // Open window 
  glutInit(argc, argv);
  glutInitWindowPosition(100, 100);
  glutInitWindowSize(GLUTwindow_width, GLUTwindow_height);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH); // | GLUT_STENCIL
  GLUTwindow = glutCreateWindow("OpenGL Viewer");

  // Initialize GLUT callback functions 
  glutIdleFunc(GLUTIdle);
  glutReshapeFunc(GLUTResize);
  glutDisplayFunc(GLUTRedrawMain);
  glutSpecialFunc(GLUTSpecial);
  glutSpecialUpFunc(GLUTSpecialUp);
  glutMotionFunc(GLUTMotion);

  // Initialize graphics modes 
  glEnable(GL_NORMALIZE);
  glEnable(GL_LIGHTING);
  glEnable(GL_DEPTH_TEST);
  glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
}

////////////////////////////////////////////////////////////
// SCENE READING
////////////////////////////////////////////////////////////


R3Scene *
ReadScene(const char *filename)
{
  // Allocate scene
  R3Scene *scene = new R3Scene();
  if (!scene) {
    fprintf(stderr, "Unable to allocate scene\n");
    return NULL;
  }

  // Read file
  if (!scene->Read(filename)) {
    fprintf(stderr, "Unable to read scene from %s\n", filename);
    return NULL;
  }

  // Remember initial camera
  camera = scene->camera;

  // Return scene
  return scene;
}



////////////////////////////////////////////////////////////
// PROGRAM ARGUMENT PARSING
////////////////////////////////////////////////////////////

int 
ParseArgs(int argc, char **argv)
{
  // Innocent until proven guilty
  int print_usage = 0;

  // Parse arguments
  argc--; argv++;
  while (argc > 0) {
    if ((*argv)[0] == '-') {
      if (!strcmp(*argv, "-help")) { print_usage = 1; }
      else if (!strcmp(*argv, "-exit_immediately")) { quit = 1; }
      else { fprintf(stderr, "Invalid program argument: %s", *argv); exit(1); }
      argv++; argc--;
    }
    else {
      if (!input_scene_name) input_scene_name = *argv;
      else { fprintf(stderr, "Invalid program argument: %s", *argv); exit(1); }
      argv++; argc--;
    }
  }

  // Check input_scene_name
  if (!input_scene_name || print_usage) {
    printf("Usage: rayview <input.scn> [-maxdepth <int>] [-v]\n");
    return 0;
  }

  // Return OK status 
  return 1;
}

////////////////////////////////////////////////////////////
// Create config map
////////////////////////////////////////////////////////////

map<char*, char*> create_config() {
 /* The code from this is heavily inspired from:
    http://www.cplusplus.com/doc/tutorial/files/
    
    But let's be honest, when you know more than 2-3 languages,
    you start mixing up how to read files.
  */

    map<char*, char*> config_map; 
   
    ifstream config_file("config.txt");
    if (config_file.is_open()) {
        while (config_file.good()) {
            char line[100] = {0};
            config_file >> line;

            if (strcmp(line, "") == 0) break;
            
            int semicolon_index = -1;
            int len = strlen(line);
            for (int i = 0; i < len; i++) {
                if (line[i] == ':') {
                    semicolon_index = i;
                    break;
                }
            }
                
            char *key = (char*) calloc(100, sizeof(char));
            char *val = (char*) calloc(100, sizeof(char));

            for (int i = 0; i < semicolon_index; i++) {
                key[i] = line[i];
            }
            int j = 0;
            for (int i = semicolon_index + 1; i < len; i++) {
                val[j++] = line[i];
            }
            config_map[key] = val;
        }
        config_file.close();
    }
    else {
        printf("unable to open file. Assuming defaults.\n");
    }

    map<char*, char*>::iterator iter;
    for (iter = config_map.begin(); iter != config_map.end(); iter++) {
        fprintf(stderr, "[%s] : [%s]\n", iter->first, iter->second);

        if (!strcmp(iter->first, "client")) {
            if (!strcmp(iter->second, "1")) {
                is_client = true;
            }
            else {
                is_client = false;
            }
        }
        
        if (!strcmp(iter->first, "ipaddress")) {
            ip_address = iter->second;
        }

        if (!strcmp(iter->first, "port")) {
            port = atoi(iter->second);
        }
        
        if (!strcmp(iter->first, "network")) {
            if (!strcmp(iter->second, "1")) {
                use_networking = true;
            }
            else {
                use_networking = false;
            }
        }
    }
    
    return config_map;
}

////////////////////////////////////////////////////////////
// MAIN
////////////////////////////////////////////////////////////

int main(int argc, char **argv) {

    //server();

    // Initialize GLUT
    GLUTInit(&argc, argv);

    // Parse program arguments
    if (!ParseArgs(argc, argv)) exit(1);

    // Create our config file mapping
    config_map = create_config();
    if (use_networking) {
        if (is_client) {
            socket_desc = client_init(ip_address, port);
            if (socket_desc != -1) { 
                fprintf(stderr, "Successful client init (probably) with socket desc %d\n", socket_desc);
                connected = true;
            }
            else {
                fprintf(stderr, "Could not init client.\n");
                connected = false;
            }
        }
        else {
            // code for initializing server.
            socket_desc = server_init(port);
            if (socket_desc != -1) {
                fprintf(stderr, "Successful server init (probably) with socket desc %d\n", socket_desc);
                connected = true;
            }
            else {
                fprintf(stderr, "Could not init server.\n");
                connected = false;
            }
        }
    }

    // Read scene
    scene = ReadScene(input_scene_name);
    if (!scene) exit(-1);

    // Run GLUT interface
    GLUTMainLoop();

    // Return success 
    return 0;
}
