//
//  main.cpp
//  DepthSensor_Buggy
//
//  Created by Azhar Aulia Saputra on 2020/05/31.
//  Copyright © 2020 Azhar Aulia Saputra. All rights reserved.
//



#ifdef _WIN32
#include <windows.h>
// Hint to hybrid-graphics drivers (NVIDIA Optimus / AMD PowerXpress) that this
// app wants the discrete GPU instead of the integrated one. The drivers scan
// the executable for these exported symbols at startup.
extern "C" {
    __declspec(dllexport) DWORD NvOptimusEnablement = 1;
    __declspec(dllexport) int   AmdPowerXpressRequestHighPerformance = 1;
}
#endif
#ifdef __APPLE__
#include <OpenGL/OpenGL.h>
#include <GLUT/GLUT.h>
#else
#include <GL/glut.h>
#include <GL/glu.h>
#endif
#if !defined(_WIN32) && !defined(__APPLE__)
// On Linux freeglut and drawstuff each own a GLX context. Creating the freeglut
// window makes its context current, stealing it from drawstuff, so we snapshot
// drawstuff's GLX context/drawable and restore it around freeglut calls — the
// same dance the _WIN32 path does with WGL.
#include <GL/glx.h>
#endif
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include "projection.h"
#include "main.h"
#include "rnd.h"
#include "extDisplay.hpp"
#include "gng.hpp"
#include "surfMatching.hpp"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

// select correct drawing functions

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif


// some constants

#define LENGTH 0.7    // chassis length
#define WIDTH 0.5    // chassis width
#define HEIGHT 0.2    // chassis height
#define RADIUS 0.18    // wheel radius
#define STARTZ 0.5    // starting height of chassis
#define CMASS 1        // chassis mass
#define WMASS 0.2    // wheel mass


#define rad 57.295779513082320876798154814105
#define phi 3.1415926535897932384626433832795
#define sp2 1.4142135623730950488016887242097


// dynamics and collision objects (chassis, 3 wheels, environment)

 dWorldID world;
 dSpaceID space;
 dBodyID body[4];
 dJointID joint[3];    // joint[0] is the front wheel
 dJointGroupID contactgroup;
 dGeomID ground;
 dSpaceID car_space;
 dGeomID box[1];
 dGeomID sphere[3];
 dGeomID ground_box;


bool sensorEnable = 1;
int startScan = 1;
// things that the user controls

double speed=0,steer=0;    // user commands


int showSimulation = 1;
// this is called by dSpaceCollide when two objects in space are
// potentially colliding.




// start simulation - set viewpoint
int captureGNG_node = 0;
static void start()
{
  dAllocateODEDataForThread(dAllocateMaskAll);

  const GLubyte *vendor   = glGetString(GL_VENDOR);
  const GLubyte *renderer = glGetString(GL_RENDERER);
  const GLubyte *version  = glGetString(GL_VERSION);
  fprintf(stderr, "[GL] vendor=%s\n[GL] renderer=%s\n[GL] version=%s\n",
          vendor   ? (const char *)vendor   : "(null)",
          renderer ? (const char *)renderer : "(null)",
          version  ? (const char *)version  : "(null)");
  fflush(stderr);

  static float xyz[3] = {-0.5,0,1.000f};
  static float hpr[3] = {0,-27.5000f,0.0000f};
  dsSetViewpoint (xyz,hpr);
  printf ("Press:\t'a' to increase speed.\n"
      "\t'z' to decrease speed.\n"
      "\t',' to steer left.\n"
      "\t'.' to steer right.\n"
      "\t' ' to reset speed and steering.\n"
      "\t'1' to save the current state to 'state.dif'.\n");
}


// called when a key pressed

int robotView = 0;
static void command (int cmd)
{
    
  switch (cmd) {
  case 'a': case 'A':
    speed += 0.3;
    break;
  case 'z': case 'Z':
    speed -= 0.3;
    break;
  case 'i':
    speed = 0.4;
    break;
  case 'k':
    speed =0;
    break;
  case ',':
    speed = -0.6;
    break;
  case 'j':
    steer = -0.6;
    break;
  case 'l':
    steer = 0.6;
    break;
  case ' ':
    robotView ++;
          if(robotView > 3) robotView = 0;
    break;
  case '1':
    printDataMap(ocnet);
    break;

      case 's':
          captureGNG_node = 1;
          tUpdate = 30;
          break;
      case 'd':
          stepMatch ++;
          break;
  }
}



dContactGeom c[100000];
MyObject pico_data[SNUM][300][600];
MyObject Sensor[SNUM];
dReal dist[SNUM][300][300];
dGeomID coob[300];
int conum = 0;


double PointCloudActive[500000][3];
int pointCloudActiveNum = 0;



int pointCloudActiveTotal = 0;

void sensorControl(){
    dVector3 sides;
    double Angle[3] = {0.35, 60/rad, -60/rad};
    for(int i=0; i<SNUM; i++){

        dReal tmp = dJointGetHingeAngle(Sensor[i].joint);
        dReal u = 10 * (Angle[i]-tmp);
        dJointSetHingeParam(Sensor[i].joint, dParamVel, u);
        dJointSetHingeParam(Sensor[i].joint, dParamFMax, 1000);
        
        dsSetColor(0,0,1.0);
        dGeomBoxGetLengths(Sensor[i].geom,sides);
        dsDrawBox(dBodyGetPosition(Sensor[i].body),
                  dBodyGetRotation(Sensor[i].body),sides);
    }
}
int inverseMatrix(const dMatrix3 R, dMatrix3 Ro){
    float a[3][3],b[3][3];
    int i,j;
    float determinant=0;
    
     for(i=0;i<3;i++){
        a[0][i] = R[0+i];
        a[1][i] = R[4+i];
        a[2][i] = R[8+i];
    }
    
    
//     for(i=0;i<3;i++)
//         determinant = determinant + (a[0][i]*(a[1][(i+1)%3]*a[2][(i+2)%3] - a[1][(i+2)%3]*a[2][(i+1)%3]));
//
//      for(i=0;i<3;i++){
//         for(j=0;j<3;j++)
//              b[i][j] = ((a[(i+1)%3][(j+1)%3] * a[(i+2)%3][(j+2)%3]) - (a[(i+1)%3][(j+2)%3]*a[(i+2)%3][(j+1)%3]))/ determinant;
//      }
    
    double det = a[0][0] * (a[1][1] * a[2][2] - a[2][1] * a[1][2]) -
                 a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
                 a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);

    double invdet = 1 / det;

//    double minv[3][3]; // inverse of matrix m
    b[0][0] = (a[1][1] * a[2][2] - a[2][1] * a[1][2]) * invdet;
    b[0][1] = (a[0][2] * a[2][1] - a[0][1] * a[2][2]) * invdet;
    b[0][2] = (a[0][1] * a[1][2] - a[0][2] * a[1][1]) * invdet;
    b[1][0] = (a[1][2] * a[2][0] - a[1][0] * a[2][2]) * invdet;
    b[1][1] = (a[0][0] * a[2][2] - a[0][2] * a[2][0]) * invdet;
    b[1][2] = (a[1][0] * a[0][2] - a[0][0] * a[1][2]) * invdet;
    b[2][0] = (a[1][0] * a[2][1] - a[2][0] * a[1][1]) * invdet;
    b[2][1] = (a[2][0] * a[0][1] - a[0][0] * a[2][1]) * invdet;
    b[2][2] = (a[0][0] * a[1][1] - a[1][0] * a[0][1]) * invdet;
    
    
//    printf("\nInverse of matrix is: \n\n");
//    for(i=0;i<3;i++){
//       for(j=0;j<3;j++)
//            printf("%.2f\t",((a[(i+1)%3][(j+1)%3] * a[(i+2)%3][(j+2)%3]) - (a[(i+1)%3][(j+2)%3]*a[(i+2)%3][(j+1)%3]))/ determinant);
//        printf("\n");
//    }
    for(i=0;i<3;i++){
        Ro[0+i] = b[0][i];
        Ro[4+i] = b[1][i];
        Ro[8+i] = b[2][i];
    }
    
//    printf("a = \n");
//    for(i=0;i<3;i++){
//        for(j=0;j<3;j++){
//            printf("%f\t",a[i][j]);
//        }
//        printf("\n");
//    }
//    printf("b = \n");
//    for(i=0;i<3;i++){
//        for(j=0;j<3;j++){
//            printf("%f\t",b[i][j]);
//        }
//        printf("\n");
//    }
    
      return 0;
}
double StartPos[3];
void sensor(int step){
    
    int h,i,j,k;
    static int flag = 1;
    double error;
    static int t = 0;
    int n = 0;
    
    int ct = 0;
    dVector3 bbpos;
    dVector3 sides;
    dReal * Pos;
//
//    if(captureGNG_node == 0){
//        for(int i = 0; i < 3; i++){
//            StartPos[i] = Posi[i];
//        }
//    }
//    printf("%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\n",Posi[0]-StartPos[0],Posi[1],Posi[2],SLAMpos[0],SLAMpos[1],SLAMpos[2]);
    dMatrix3 R_l;
    dRFromAxisAndAngle(R_l ,0,0, 1, -M_PI/2);

    
    
    pointCloudActiveNum = 0;
    int ScanNumX = 1;
    int ScanNumY = 150;
    for (int i=0; i<3; i++) sides[i] = 0.005;
    int pointCount = 0;
//    for()
    double aty = 0;
    double atx = 0;
    int countY = 0;
    
    const dReal * Posiii = dBodyGetPosition(Sensor[0].body);
    const dReal * Rot = dBodyGetRotation(Sensor[0].body);
    const dReal * RotBody = dBodyGetRotation(body[0]);
    dMatrix3 RR;
    inverseMatrix(RotBody, RR);
//    double z = atan2(RR[4], RR[0]);
//    double z1 = atan2(RotBody[4], RotBody[0]);
//    double rot_z = atan2(Rot[4], Rot[0]);
    for(i=0; i<SNUM; i++){

        const dReal * Rot = dBodyGetRotation(Sensor[i].body);
        const dReal * Posi = dBodyGetPosition(Sensor[i].body);
        for(aty = yMin; aty < yMin+(yMax-yMin); aty+=(yMax-yMin)/ScanNumY){
            countY++;
            for(atx = xMin; atx < xMin + (xMax-xMin)/xNUM; atx+=(xMax-xMin)/(ScanNumX*xNUM)){
        
        for(j=0; j<yNUM; j++){
//            double angle = aty + M_PI + M_PI/6 + 1;
            double angle = aty;
            for(k=0; k<xNUM; k++){
                ct = 0;
                dMatrix3 Ra,Rb,Rc;
                
                double angle2 = -0.58+k*(1.16/(double)(xNUM));
                double rot0[3] = {1,angle2, angle};
//                double rot0[3] = {1,0, 0};
                double rot[3] = {1,0, 0};
                double pos[3] = {Posi[0],Posi[1], Posi[2]};
//                double pos[3] = {0,0,1};
                multiplicationRotation(Rot, rot0, rot);
//
                dGeomRaySet(pico_data[i][j][k].geom, pos[0],pos[1], pos[2], rot[0], rot[1], rot[2]);
                
                
//                if(countY%5 == 0 && k%5 == 0){
//                    dVector3 end,start,direction;
//
//                    dGeomRayGet(pico_data[i][j][k].geom, start, direction);
//                    for(int q = 0; q < 3; q++ ){
//                        direction[q] = rot[q];
//                        start[q] = pos[q];
//                    }
//                    end[0] = start[0] + (direction[0]*2);
//                    end[1] = start[1] + (direction[1]*2);
//                    end[2] = start[2] + (direction[2]*2);
//                    end[3] = start[3] + (direction[3]*2);
//
//                    dsSetColorAlpha(1.0, 1.0, 0.0, 1.5);
//                    dsDrawLineD(start, end);
//                }
                
//                dRFromAxisAndAngle ()
//                multiplicationRotation(Rc, Ra, Rb);
//                dGeomSetRotation(pico_data[i][j][k].geom,Ra);
                
                for(h=0;h<conum;h++){
                    int numc = dCollide(pico_data[i][j][k].geom, coob[h], 1, &c[i], sizeof( dContact ) );
                    if (numc > 0) {
                        if(h == 0){
                            dist[i][j][k] = c[i].depth; // depthはrayの始点位置からの距離
                            Pos = c[i].pos;
                            for (int i=0; i<3; i++) bbpos[i] = Pos[i];
                        }
                        else if(ct == 0){
                            dist[i][j][k] = c[i].depth;
                            Pos = c[i].pos;
                            for (int i=0; i<3; i++) bbpos[i] = Pos[i];
                        }else if(dist[i][j][k] > c[i].depth){
                            dist[i][j][k] = c[i].depth;
                            Pos = c[i].pos;
                            for (int i=0; i<3; i++) bbpos[i] = Pos[i];
                        }
                        ct++;
                    }
                }
                if(ct == 0){
                    dist[i][j][k] = 0.0;
                    for (int i=0; i<3; i++) bbpos[i] = 0;
                }
                if(showSimulation && countY%5 == 0 && k%5 == 0){
                    dMatrix3 RI;
                    dRSetIdentity (RI);
                    dsSetColor(1,0,0);
                    dsDrawBox(bbpos,RI,sides);
                }
                if(startScan && ct > 0){
                    for (int i=0; i<3; i++)
//                    PointCloudScan[step][pointCloudActiveNum][i] = bbpos[i];
                        PointCloudActive[pointCloudActiveNum][i] = bbpos[i] - Posiii[i];// + SLAMpos[i];
                    double r[3];
                    multiplicationRotation(RR, PointCloudActive[pointCloudActiveNum], r);
                    for (int i=0; i<3; i++)
                        PointCloudActive[pointCloudActiveNum][i] = r[i];
                    pointCloudActiveNum++;
                }
            }
        }
    }
    }
    }
    pointCloudActiveTotal = pointCloudActiveNum;
}

void makeDataSensor(){

    dMass m;
    
    double posSens[SNUM][3] = {
        {0,0,STARTZ+0.3},
        {0,-0.03,STARTZ+0.3},
        {0,0.03,STARTZ+0.3},
    };
    for(int k=0; k < 3; k++)
        SLAMpos[k] = posSens[0][k];
    
    for(int i=0; i<SNUM; i++){
        Sensor[i].body = dBodyCreate (world);
        dBodySetPosition (Sensor[i].body,posSens[i][0],posSens[i][1],posSens[i][2]);
        dMassSetBox (&m,1,0.01,0.02,0.04);
        dMassAdjust (&m,0.01);
        dBodySetMass (Sensor[i].body,&m);
        Sensor[i].geom = dCreateBox (0,0.01,0.02,0.04);
        dGeomSetBody (Sensor[i].geom,Sensor[0].body);
    }
    Sensor[0].joint = dJointCreateHinge(world, 0);
    dJointAttach(Sensor[0].joint, Sensor[0].body, body[0]);
    dJointSetHingeAnchor (Sensor[0].joint,LENGTH/2,0,STARTZ+0.3);
    dJointSetHingeAxis (Sensor[0].joint,0,1,0);
    
    for(int i=1; i<SNUM; i++){
        Sensor[i].joint = dJointCreateHinge(world, 0);
        dJointAttach(Sensor[i].joint, Sensor[i].body, Sensor[0].body);
        dJointSetHingeAnchor (Sensor[i].joint,posSens[i][0]-0.02,posSens[i][1],posSens[i][2]);
        dJointSetHingeAxis (Sensor[i].joint,0,0,1);
    }
    
    int i,j,k;
    double *pos = (double *) malloc(sizeof (double) * 3);
    double *posz = (double *) malloc(sizeof (double) * 3);
    double *pos1 = (double *) malloc(sizeof (double) * 3);
    pos[0] = 0;
    pos[1] = 0;
    pos[2] = 1;
    
    pos1[1] = 0;
    float yRes = (yMax - yMin)/(yNUM-1);
    for(i=0; i<SNUM; i++){
        for(j=0; j<yNUM; j++){
            
            if(yNUM == 1)
                pos1[2] = tan(yMin+(yMax - yMin)/2);
            else
                pos1[2] = tan(yMin+j*yRes-M_PI/4);
            
            double angle = yMin+(double)j*yRes + M_PI - M_PI/6;
            if(yNUM == 1)
                angle = yMin + M_PI - M_PI/6;
            
            pos1 = rotation_x(angle, pos);
            for(k=0; k<xNUM; k++){
                double angle2 = xMin+k*xRes;
                pico_data[i][j][k].geom = dCreateRay(space, 10);
                dGeomRaySet(pico_data[i][j][k].geom, 0, 0, 0.2, 1, angle2, 0);
//                dGeomSetPosition(pico_data[i][j][k].geom , 0, 0, 0.7);
//                dGeomSetBody(pico_data[i][j][k].geom, pico[i].body);
//                dGeomSetOffsetPosition(pico_data[i][j][k].geom, 0.0, 0.0, 0.0);
                dMatrix3 R_l;
//                pos1[0] = tan(xMin+k*xRes);
//                pos1 = rotation_z(xMin+(double)k*xRes, pos);
                
//                float *pos1 = rotation_y((-121+(yRes*(dReal)j))/rad, pos);
//                printf("%.2f\t%.2f\t%.2f\n", pos1[0], pos1[1], pos1[2]);
//                float *pos2 = rotation_z(0, pos1);
//                printf("%.2f\t%.2f\t%.2f\n", pos2[0], pos2[1], pos2[2]);
                
                
                double *pos3 = crossProductD(pos, pos1);
                float ang = acos((pos[0]*pos1[0] + pos[1]*pos1[1] + pos[2]*pos1[2])/(norm(pos, 3) * norm(pos1, 3)));
                
//                dRFrom2Axes(R_l, 0.0, 0.0, 0.0, (-22.5+(xRes*(dReal)k))/rad, 0.0, (-22.5+(xRes*(dReal)k))/rad);
//                dRFromEulerAngles(R_l , cos((121-(yRes*(dReal)j))/rad)*(-22.5+(xRes*(dReal)k))/rad, (121-(yRes*(dReal)j))/rad, sin((121-(yRes*(dReal)j))/rad)*(-22.5+(xRes*(dReal)k))/rad);
//                dGeomSetOffsetRotation(pico_data[i][j][k].geom,R_l);
//                printf("%.2f\t%.2f\t%.2f\n",pos1[0],pos1[1],pos1[2]);
//                printf("%.2f\t%.2f\t%.2f\n",pos3[0],pos3[1],pos3[2]);
//                dRFromAxisAndAngle(R_l ,pos3[0], pos3[1], pos3[2], ang);
//                dRFrom2Axes (R_l, pos[0],pos[1],pos[2],pos1[0],pos1[1],pos1[2]);
//                dRFromEulerAngles (R_l,angle, angle2,0);
//                dRFromAxisAndAngle(R_l ,0,1,0, 0.1);
//                dGeomSetOffsetRotation(pico_data[i][j][k].geom,R_l);
//                dGeomSetRotation(pico_data[i][j][k].geom,R_l);
            }
        }
    }
}

void drawDataSensor(){
    int i,j,k;
    for(i=0; i<SNUM; i++){
        if(i == 0){
            dsSetColorAlpha(1.0, 1.0, 0.0, 0.5);
        }
        if(i == 1){
            dsSetColorAlpha(0.5, 1.0, 0.5, 0.5);
        }
        if(i == 2){
            dsSetColorAlpha(0.1, 0.3, 0.3, 0.5);
        }
        if(i == 3){
            dsSetColorAlpha(0.3, 0.3, 0.8, 0.5);
        }
        for(j=0; j<yNUM; j+=1){
            for(k=0; k<xNUM; k+=1){
                
                dVector3 start,direction,end;
                dGeomRayGet(pico_data[i][j][k].geom, start, direction);
                dReal length = dGeomRayGetLength(pico_data[i][j][k].geom);
                
                end[0] = start[0] + (direction[0]*length);
                end[1] = start[1] + (direction[1]*length);
                end[2] = start[2] + (direction[2]*length);
                end[3] = start[3] + (direction[3]*length);
                
//                dsDrawLineD(start, end);
            }
        }
    }
}


void drawRobot(){

    int i;
    dsSetColor (0,1,1);
    dsSetTexture (DS_WOOD);
    dReal sides[3] = {LENGTH,WIDTH,HEIGHT};
    dsDrawBox (dBodyGetPosition(body[0]),dBodyGetRotation(body[0]),sides);
    dsSetColor (1,1,1);
    for (i=1; i<=3; i++) dsDrawCylinder (dBodyGetPosition(body[i]),
                         dBodyGetRotation(body[i]),0.02f,RADIUS);
}
void makeRobot(){

    int i;
    dMass m;
    
    // chassis body
    body[0] = dBodyCreate (world);
    dBodySetPosition (body[0],0,0,STARTZ);
    dMassSetBox (&m,1,LENGTH,WIDTH,HEIGHT);
    dMassAdjust (&m,CMASS);
    dBodySetMass (body[0],&m);
    box[0] = dCreateBox (0,LENGTH,WIDTH,HEIGHT);
    dGeomSetBody (box[0],body[0]);

    // wheel bodies
    for (i=1; i<=3; i++) {
      body[i] = dBodyCreate (world);
      dQuaternion q;
      dQFromAxisAndAngle (q,1,0,0,M_PI*0.5);
      dBodySetQuaternion (body[i],q);
        dMassSetCylinder (&m,1, 1,RADIUS, 0.02);
      dMassAdjust (&m,WMASS);
      dBodySetMass (body[i],&m);
        sphere[i-1] = dCreateCylinder(0,RADIUS, 0.02);
      dGeomSetBody (sphere[i-1],body[i]);
    }
    dBodySetPosition (body[1],0.5*LENGTH,0,STARTZ-HEIGHT*0.5);
    dBodySetPosition (body[2],-0.5*LENGTH, WIDTH*0.5,STARTZ-HEIGHT*0.5);
    dBodySetPosition (body[3],-0.5*LENGTH,-WIDTH*0.5,STARTZ-HEIGHT*0.5);

    // front and back wheel hinges
    for (i=0; i<3; i++) {
      joint[i] = dJointCreateHinge2 (world,0);
      dJointAttach (joint[i],body[0],body[i+1]);
      const dReal *a = dBodyGetPosition (body[i+1]);
      dJointSetHinge2Anchor (joint[i],a[0],a[1],a[2]);
      dJointSetHinge2Axis1 (joint[i],0,0,1);
      dJointSetHinge2Axis2 (joint[i],0,1,0);
    }

    // set joint suspension
//    for (i=0; i<3; i++) {
//      dJointSetHinge2Param (joint[i],dParamSuspensionERP,0.4);
//      dJointSetHinge2Param (joint[i],dParamSuspensionCFM,0.8);
//    }

    // lock back wheels along the steering axis
    for (i=1; i<3; i++) {
      // set stops to make sure wheels always stay in alignment
      dJointSetHinge2Param (joint[i],dParamLoStop,0);
      dJointSetHinge2Param (joint[i],dParamHiStop,0);
      // the following alternative method is no good as the wheels may get out
      // of alignment:
      //   dJointSetHinge2Param (joint[i],dParamVel,0);
      //   dJointSetHinge2Param (joint[i],dParamFMax,dInfinity);
    }

    // create car space and add it to the top level space
    car_space = dSimpleSpaceCreate (space);
    dSpaceSetCleanup (car_space,0);
    dSpaceAdd (car_space,box[0]);
    dSpaceAdd (car_space,sphere[0]);
    dSpaceAdd (car_space,sphere[1]);
    dSpaceAdd (car_space,sphere[2]);

}
// simulation loop
void controlBuggy(){
    dJointSetHinge2Param (joint[0],dParamVel2,-speed);
    dJointSetHinge2Param (joint[0],dParamFMax2,0.1);

    // steering
    dReal v = steer - dJointGetHinge2Angle1 (joint[0]);
    if (v > 0.1) v = 0.1;
    if (v < -0.1) v = -0.1;
    v *= 10.0;
    dJointSetHinge2Param (joint[0],dParamVel,v);
    dJointSetHinge2Param (joint[0],dParamFMax,0.2);
    dJointSetHinge2Param (joint[0],dParamLoStop,-0.75);
    dJointSetHinge2Param (joint[0],dParamHiStop,0.75);
    dJointSetHinge2Param (joint[0],dParamFudgeFactor,0.1);
    
//    if(speed < -0.01)
//        speed += 0.01;
//    else if(speed > 0.01)
//        speed -= 0.01;
//    else speed = 0;
//
    if(steer < -0.01)
        steer += 0.01;
    else if(steer > 0.01)
        steer -= 0.01;
    else steer = 0;
}
static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
     int i, j;
  //int c;
  static const int N = 100;
  double pos[3], d;
  dContact contact[N];
  
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnected(b1, b2)) return;
  
  int check = 0;
  int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));

  if (n > 0) {
      int i,j,k;
      if(sensorEnable)
          for(i=0; i<SNUM; i++){
              for(j=0; j<yNUM; j++){
                  for(k=0; k<xNUM; k++){
                      if (o1==pico_data[i][j][k].geom || o2==pico_data[i][j][k].geom) {
                          check = 1;
                      }
                      if(check == 1) break;
                  }
              }
          }
      if(check == 0){
          for (int i = 0; i < n; i++) {
              contact[i].surface.mode = dContactSlip1 | dContactSlip2 | dContactSoftERP | dContactSoftCFM | dContactApprox1;
              contact[i].surface.mu = dInfinity;
              contact[i].surface.slip1 = 0.005;
              contact[i].surface.slip2 = 0.01;
              contact[i].surface.soft_erp = 0.1;
              contact[i].surface.soft_cfm = 1e-4;
              dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
              dJointAttach(c, dGeomGetBody(contact[i].geom.g1),dGeomGetBody(contact[i].geom.g2));
          }
      }
  }
}
bool initGL = 1;
void rotationMatrixToEulerAngles(float R[12], float Rot[3])
{
    float x,y,z;
    
//    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
//
//    bool singular = sy < 1e-6; // If
//
//    float x, y, z;
//    if (!singular)
//    {
//        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
//        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R[4], R[0]);
//    }
//    else
//    {
//        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
//        y = atan2(-R.at<double>(2,0), sy);
//        z = 0;
//    }
//    return Vec3f(x, y, z);
    Rot[0] = z * 180/M_PI;
    
}
double sensorPos[3];
void controlView(){

    const dReal* Pos = dBodyGetPosition(Sensor[0].body);
    
    memcpy(sensorPos, Pos, 3);
    
    for(int i=0; i<3; i++)
        sensorPos[i] = Pos[i];
    
    static int flagView = 0;
    if(robotView == 0){
        const dReal* Pos = dBodyGetPosition(body[0]);
        const dReal* Rot = dBodyGetRotation(body[0]);
        float Rot1[12], RotE[3] ={0.0,-37,0.0};
        for(int i = 0; i < 12; i++){
            Rot1[i] = Rot[i];
        }
        rotationMatrixToEulerAngles(Rot1,RotE);
        float xyz[3] = {static_cast<float>(Pos[0]) + 0.2f,static_cast<float>(Pos[1]),static_cast<float>(Pos[2]) + 1.5f};
    //        static float hpr[3] = {Pos[0],Pos[1],Pos[2]};
            
        dsSetViewpoint (xyz,RotE);
        updateGLView(RotE, xyz);
        flagView = 0;
    }else if(robotView == 1){
        if(flagView == 0){
            flagView = 1;
            const dReal* Pos = dBodyGetPosition(body[0]);
            const dReal* Rot = dBodyGetRotation(body[0]);
            float Rot1[12], RotE[3] ={-0.7,0,0.7};
            for(int i = 0; i < 12; i++){
                Rot1[i] = Rot[i];
            }
            rotationMatrixToEulerAngles(Rot1,RotE);
            float xyz[3] = {static_cast<float>(Pos[0]) - 0.7f,static_cast<float>(Pos[1]),static_cast<float>(Pos[2]) + 0.7f};
        //        static float hpr[3] = {Pos[0],Pos[1],Pos[2]};
            dsSetViewpoint (xyz,RotE);
            updateGLView(RotE, xyz);
        }else{
            float xyz[3], hpr[3];
            dsGetViewpoint(xyz, hpr);
            updateGLView(hpr, xyz);
        }
    }else if(robotView == 2){
        static float xyz[3] = {2,0,5};
        static float hpr[3] = {0,-90,0};
        xyz[0] = SLAMpos[0];
        xyz[1] = SLAMpos[1];
        dsSetViewpoint (xyz,hpr);
        updateGLView(hpr, xyz);
        flagView = 0;
    }else if(robotView == 3){
        float xyz[3], hpr[3];
        dsGetViewpoint(xyz, hpr);
        updateGLView(hpr, xyz);
    }
}
//int tUpdate;
#ifdef _WIN32
static HGLRC g_ds_ctx = nullptr;
static HDC   g_ds_dc  = nullptr;
#endif
#if !defined(_WIN32) && !defined(__APPLE__)
static Display*    g_ds_dpy  = nullptr;
static GLXDrawable g_ds_draw = 0;
static GLXContext  g_ds_glx  = nullptr;
#endif

static void simLoop (int pause)
{
    if(initGL){
#ifdef _WIN32
        // Snapshot drawstuff's WGL context before freeglut creates its own
        // window and switches the current context away from us.
        g_ds_dc  = wglGetCurrentDC();
        g_ds_ctx = wglGetCurrentContext();
#endif
#if !defined(_WIN32) && !defined(__APPLE__)
        g_ds_dpy  = glXGetCurrentDisplay();
        g_ds_draw = glXGetCurrentDrawable();
        g_ds_glx  = glXGetCurrentContext();
#endif
        initGL = 0;
        initialExtGL();
#ifdef _WIN32
        if (g_ds_dc && g_ds_ctx) wglMakeCurrent(g_ds_dc, g_ds_ctx);
#endif
#if !defined(_WIN32) && !defined(__APPLE__)
        if (g_ds_dpy && g_ds_glx)
            glXMakeCurrent(g_ds_dpy, g_ds_draw, g_ds_glx);
#endif
    }
    if (!pause) {
    // motor
        controlBuggy();
        sensorControl();
        dSpaceCollide (space,0,&nearCallback);
        dWorldStep (world,0.1);

        // remove all contact joints
        dJointGroupEmpty (contactgroup);
    }

    drawRobot();
    drawEnvironment();
    controlView();

//    if(sensorEnable)
//        drawDataSensor();

#ifdef _WIN32
    // Drive freeglut: timer/display callbacks (which run the GNG pipeline and
    // paint the secondary window) only fire from inside an event-loop call.
    // Freeglut leaves its own context current after callbacks, so restore
    // drawstuff's for the next frame.
    glutMainLoopEvent();
    if (g_ds_dc && g_ds_ctx) wglMakeCurrent(g_ds_dc, g_ds_ctx);
#endif
#if !defined(_WIN32) && !defined(__APPLE__)
    // Same as the _WIN32 path: pump freeglut so the GNG display()/cal_env
    // callbacks run and the secondary window paints, then hand the current
    // context back to drawstuff for its buffer swap.
    //
    // Throttle the pump: drawstuff runs at full speed (~100 fps), but pumping
    // freeglut every frame let the GNG window redraw ~200x/s, which under GNOME
    // turned into an expose-storm — the GNG window kept being raised to the
    // front, stealing the mouse from the drawstuff window (camera felt frozen)
    // and visibly flickering. Pumping every 6th frame keeps the GNG view at a
    // calm ~15 Hz and lets drawstuff own input the rest of the time.
    static int pumpDiv = 0;
    if (++pumpDiv >= 6) {
        pumpDiv = 0;
        glutMainLoopEvent();
        if (g_ds_dpy && g_ds_glx)
            glXMakeCurrent(g_ds_dpy, g_ds_draw, g_ds_glx);
    }
#endif
}

//int main (int argc, char **argv)
//{
//    printf("test");
//    return 0;
//}
int main (int argc, char **argv)
{
//    while(1){
//        double n[3] = {2,1,2};
//        n[0] = rnd();
//        n[1] = rnd();
//        n[2] = rnd();
//        double n2[3] = {2,1,2};
//        double a = norm(n, 3);
////        double *b = vectorSubtraction(n, n2, 3);
//        double *c = vectorScale(a, vectorSubtraction(n, n2, 3), 3);
//        double *e = vectorScale(a, c, 3);
//        vectorScaleD(a, c, 3, c);
////        for(int i=0; i<3;i++){
////            b[i] = e[i];
////        }
////        double *d = vectorAdd(c,c,3);
//        vectorUnitD(n,n2);
//        free(c);
////        free(d);
//        free(e);
//    }
//    vectorToEulerTest();
    // setup pointers to drawstuff callback functions
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &simLoop;
    fn.command = &command;
    fn.stop = 0;
    fn.path_to_textures = "textures";

    // create world

    dInitODE2(0);
    world = dWorldCreate();
    space = dHashSpaceCreate (0);
    contactgroup = dJointGroupCreate (0);
    dWorldSetGravity (world,0,0,-0.5);
    ground = dCreatePlane (space,0,0,1,0);

    makeRobot();
    makeEnvironment();
    if(sensorEnable){
        makeDataSensor();
    }
    glutInitWindowPosition(0, 0);
    // run simulation
    dsSimulationLoop (argc,argv,600,800,&fn);

    dGeomDestroy (box[0]);
    dGeomDestroy (sphere[0]);
    dGeomDestroy (sphere[1]);
    dGeomDestroy (sphere[2]);
    dJointGroupDestroy (contactgroup);
    dSpaceDestroy (space);
    dWorldDestroy (world);
    dCloseODE();
    return 0;
}


void Arrow(GLdouble x1,GLdouble y1,GLdouble z1,GLdouble x2,GLdouble y2,GLdouble z2,GLdouble D)
{
    double x=x2-x1;
    double y=y2-y1;
    double z=z2-z1;
    double L=sqrt(x*x+y*y+z*z);
    
    GLUquadricObj *quadObj;
    
    glPushMatrix ();
    
    glTranslated(x1,y1,z1);
    if((x!=0.)||(y!=0.)) {
        glRotated(atan2(y,x)/deg_to_rad,0.,0.,1.);
        glRotated(atan2(sqrt(x*x+y*y),z)/deg_to_rad,0.,1.,0.);
    } else if (z<0){
        glRotated(180,1.,0.,0.);
    }
    
    glTranslatef(0,0,L-4*D);
    
    quadObj = gluNewQuadric ();
    gluQuadricDrawStyle (quadObj, GLU_FILL);
    gluQuadricNormals (quadObj, GLU_SMOOTH);
    gluCylinder(quadObj, 2*D, 0.0, 4*D, 32, 1);
    gluDeleteQuadric(quadObj);
    
    quadObj = gluNewQuadric ();
    gluQuadricDrawStyle (quadObj, GLU_FILL);
    gluQuadricNormals (quadObj, GLU_SMOOTH);
    gluDisk(quadObj, 0.0, 2*D, 32, 1);
    gluDeleteQuadric(quadObj);
    
    glTranslatef(0,0,-L+4*D);
    
    quadObj = gluNewQuadric ();
    gluQuadricDrawStyle (quadObj, GLU_FILL);
    gluQuadricNormals (quadObj, GLU_SMOOTH);
    gluCylinder(quadObj, D, D, L-4*D, 32, 1);
    gluDeleteQuadric(quadObj);
    
    quadObj = gluNewQuadric ();
    gluQuadricDrawStyle (quadObj, GLU_FILL);
    gluQuadricNormals (quadObj, GLU_SMOOTH);
    gluDisk(quadObj, 0.0, D, 32, 1);
    gluDeleteQuadric(quadObj);
    
    glPopMatrix ();
    
}
