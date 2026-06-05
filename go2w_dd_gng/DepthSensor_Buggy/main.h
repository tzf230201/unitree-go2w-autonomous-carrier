//
//  main.h
//  DepthSensor_Buggy
//
//  Created by Azhar Aulia Saputra on 2020/08/18.
//  Copyright © 2020 Azhar Aulia Saputra. All rights reserved.
//

#ifndef main_h
#define main_h
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#undef near
#undef far
#endif
#ifdef __APPLE__
#include <OpenGL/OpenGL.h>
#include <GLUT/GLUT.h>
#else
// freeglut.h pulls in the standard GLUT API plus extensions like
// glutMainLoopEvent, which we use to pump the secondary window.
#include <GL/freeglut.h>
#include <GL/glu.h>
#endif
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif


#define RAD 180.0/3.14159265358979323846264338327950288
#define deg_to_rad    (float)0.0174532925199432957692369076848
#define rad_to_deg    (float)57.295779513082320876798154814105
#define rad 57.295779513082320876798154814105
#define phi 3.1415926535897932384626433832795
#define sp2 1.4142135623730950488016887242097

#define SNUM 3
#define GAIN 300
#define yNUM 1
#define xNUM 150

#define yMin -M_PI/4
//#define yMin 0.0
#define yMax M_PI/4
//#define yMax M_PI/2
#define xMin -0.41421
#define xMax 0.41421
//#define xMin -M_PI/2
//#define xMax M_PI/2

//#define yRes (yMax - yMin)/(yNUM-1)
#define xRes (xMax - xMin)/(xNUM-1)

typedef struct {
    
    dBodyID body;
    dGeomID geom;
    dJointID joint;
    dReal size[3];
    dReal weight;
    dReal position[3];
    
} MyObject;
extern dWorldID world;
extern dGeomID ground;
extern dSpaceID space;
extern dJointGroupID contactgroup;
extern dsFunctions fn;
extern bool sensorEnable;
extern double sensorPos[3];
extern int captureGNG_node;
extern double speed;

extern dGeomID coob[300];
extern MyObject Sensor[SNUM];
extern int conum;
extern int showSimulation;
extern double PointCloudActive[500000][3];
extern int pointCloudActiveNum;
extern int pointCloudActiveTotal;
void makeEnvironment();
void drawEnvironment();
void Arrow(GLdouble x1,GLdouble y1,GLdouble z1,GLdouble x2,GLdouble y2,GLdouble z2,GLdouble D);
void sensor(int step);
extern int tUpdate;
extern double testAngle;


#endif /* main_h */
