//show the behavior of diabling resize callback and move the defining of viewport and stuff into display callback
//show change of glTranslatef(0, 0, -10)
//show change of glVertex3f
//show change of idle function, diable the loadIdentity in the display function
#include "cos426_opengl.h"
#include <stdio.h>



#ifdef _WIN32
#  include <windows.h>
#else
#  include <sys/time.h>
#endif

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

static int spin = 0;
double width;
double height;
char time_str[50];
char lap_str[50];
int lap = 1;
int mainWindow = 0;
int subWindow = 0;

void init(void) 
{
   GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
   GLfloat mat_shininess[] = { 50.0 };
   GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 };
   glClearColor (0.0, 0.0, 0.0, 0.0);
   glShadeModel (GL_SMOOTH);

   glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
   glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
   glLightfv(GL_LIGHT0, GL_POSITION, light_position);


  GLfloat light1_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
  GLfloat light1_specular[] = { 1.0, 1.0, 1.0, 1.0 };

  glLightfv(GL_LIGHT1, GL_DIFFUSE, light1_diffuse);
  glLightfv(GL_LIGHT1, GL_SPECULAR, light1_specular);
  glLightfv(GL_LIGHT1, GL_POSITION, light_position);

   glEnable(GL_LIGHTING);
   glEnable(GL_LIGHT0);
   glEnable(GL_LIGHT1);

   glEnable(GL_DEPTH_TEST);

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
    if (curtime - prevtime > 0.01) {
      sprintf(time_str, "Time: %4.1f", curtime);
      prevtime = curtime;
    }

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

void drawEnd(char* text)
{
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0.0, 500, 500, 0.0, -1.0, 10.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glDisable(GL_LIGHTING); 
    glColor3f(0.0f, 0.0f, 1.0f);
    GLUTPrint(220, 375, "Replay");
    glColor3f(1.0f, 1.0f, 1.0f);

    GLUTPrint(225, 225, text);
    glBegin(GL_QUADS);
        glVertex2f(200, 350); // vertex 1
        glVertex2f(200, 400); // vertex 2
        glVertex2f(300, 400); // vertex 3
        glVertex2f(300, 350); // vertex 4
    glEnd();

    glEnable(GL_LIGHTING); 
    
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);  
}



void displayEnd(void){
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   glViewport(0, 0, width, height);
   gluPerspective(45, (width)/height, 1, 2000);
   glMatrixMode(GL_MODELVIEW);

   glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

   glPushMatrix ();
   glTranslatef (0.0, 0.0, -5.0);
   drawEnd("END!");
   glPopMatrix ();
   glFlush ();
}

void displaySubwindow(void) {
   glutSetWindow(subWindow);
 
  if (lap == 3) {
    return;
  }

   GLfloat position[] = { 0.0, 0.0, 1.5, 1.0 };

   glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

   glEnable(GL_LIGHTING);
   glPushMatrix ();
   glTranslatef (0.0, 0.0, -5.0);

   glPushMatrix ();
   glRotated ((GLdouble) spin, 1.0, 0.0, 0.0);
   glLightfv (GL_LIGHT0, GL_POSITION, position);

   glTranslated (0.0, 0.0, 1.5);
   glDisable (GL_LIGHTING);
   glColor3f (0.0, 1.0, 1.0);
   glutWireCube (0.1);
   glEnable (GL_LIGHTING);
   glPopMatrix ();

   glutSolidTorus (0.275, 0.85, 8, 15);

   glPopMatrix ();

   glFlush ();  
   glutPostRedisplay();
}

void display(void)
{
   if (lap == 3) {
    displayEnd();
    return;
   }
   GLfloat position[] = { 0.0, 0.0, 1.5, 1.0 };

   glEnable(GL_SCISSOR_TEST);
   glEnable(GL_DEPTH_TEST);
   GLfloat color[4] = {1.0, 0.0, 0.0, 1.0};

   // main view port
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   glViewport(0, 0, width, height);
   gluPerspective(45, (width)/height, 1, 2000);
   glMatrixMode(GL_MODELVIEW);
   glScissor(0, 0, width , height);
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
   glLoadIdentity();
   gluLookAt( 0.0, 0.0, 50.0,
              0.0, 0.0, 0.0,
              0.0, 1.0, 0.0 );

   glPushMatrix ();
   glRotated ((GLdouble) spin, 1.0, 0.0, 0.0);
   glLightfv (GL_LIGHT0, GL_POSITION, position);

   glTranslated (0.0, 0.0, 1.5);
   glDisable (GL_LIGHTING);
   glColor3f (0.0, 1.0, 1.0);
   glutWireCube (0.1);
   glEnable (GL_LIGHTING);
   glPopMatrix ();

   glutSolidTorus (0.275, 0.85, 8, 15);
   drawHUD();

   glPopMatrix ();


   /*
    * THE SECOND VIEW PORT
    */
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   glViewport(2*width/3, 0, width/3 , height/2);
   gluPerspective(45, (width)/2/(height), 1, 2000);
   glMatrixMode(GL_MODELVIEW);
   glScissor((2*width/3), 0, width/3 , height/2);
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
   glLoadIdentity();
   gluLookAt( 0.0, 0.0, 50.0,
              0.0, 0.0, 0.0,
              0.0, 1.0, 0.0 );

   glPushMatrix ();
   glRotated ((GLdouble) spin, 1.0, 0.0, 0.0);
   glLightfv (GL_LIGHT0, GL_POSITION, position);

   glTranslated (0.0, 0.0, 1.5);
   glDisable (GL_LIGHTING);
   glColor3f (0.0, 1.0, 1.0);
   glutWireCube (0.1);
   glEnable (GL_LIGHTING);
   glPopMatrix ();

   glutSolidTorus (0.275, 0.85, 8, 15);

   glDisable(GL_SCISSOR_TEST);
   glFlush();
   glutSwapBuffers();
   glutPostRedisplay();

}



void reshape (int w, int h)
{
   width = w;
   height = h;
   glViewport (0, 0, (GLsizei) w, (GLsizei) h);
   glMatrixMode (GL_PROJECTION);
   glLoadIdentity();
   gluPerspective(40.0, (GLfloat) w/(GLfloat) h, 1.0, 20.0);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
}

void mouse(int button, int state, int x, int y)
{
   switch (button) {
      case GLUT_LEFT_BUTTON:
         if (lap == 3 && x < 300 && x > 200 && y < 400 && y > 350) {
          lap = 1;
          spin = 0;
          break;
         } 
         if (state == GLUT_DOWN) {
            spin = (spin + 30) % 360;
            if (spin == 0) lap++;
            glutPostRedisplay();
         }
         break;
      default:
         break;
   }
}

int main(int argc, char** argv)
{
   glutInit(&argc, argv);
   glutInitDisplayMode (GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
   glutInitWindowSize (500, 500); 
   glutInitWindowPosition (100, 100);
   mainWindow = glutCreateWindow (argv[0]);
   init ();
   glutDisplayFunc(display); 
   glutReshapeFunc(reshape);
   glutMouseFunc(mouse);
   glutSetWindow(mainWindow);

   glutMainLoop();
   return 0;
}