//
//  environment.cpp
//  DepthSensor_Buggy
//
//  Created by Azhar Aulia Saputra on 2020/08/18.
//  Copyright © 2020 Azhar Aulia Saputra. All rights reserved.
//

#include <stdio.h>

#ifdef _WIN32
#include <windows.h>
#endif
#ifdef __APPLE__
#include <OpenGL/OpenGL.h>
#include <GLUT/GLUT.h>
#else
#include <GL/glut.h>
#include <GL/glu.h>
#endif
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include "main.h"

#define WALL_NUM 16
#define OBJ_NUM 5

dGeomID wall[WALL_NUM];
dGeomID object[OBJ_NUM];

float wallSize[WALL_NUM][3] = {
    {1.2, 0.01, 1.5},
    {1.2, 0.01, 1.5},
    {1.2, 0.01, 1.5},
    {1.2, 0.01, 1.5},
    {4, 0.01, 1.5},
    {4, 0.01, 1.5},
    {0.01, 2, 1.5},
    {0.01, 2, 1.5},
    {0.01, 2, 1.5},
    {0.01, 2, 1.5},
    {0.01, 2, 1.5},
    {0.01, 2, 1.5},
    {0.01, 2, 1.5},
    {0.01, 2, 1.5},
    {0.01, 10, 1.5},
    {0.01, 10, 1.5},
};
float wallPos[WALL_NUM][3] = {
    {1.4,   1, wallSize[0][2]/2},
    {-1.4,  1, wallSize[0][2]/2},
    {1.4,   3, wallSize[0][2]/2},
    {-1.4,  3, wallSize[0][2]/2},
    {0,     -1, wallSize[0][2]/2},
    {0,     5, wallSize[0][2]/2},
    {0.8,   2, wallSize[0][2]/2},
    {-0.8,  2, wallSize[0][2]/2},
    {2,     6, wallSize[0][2]/2},
    {2,     2, wallSize[0][2]/2},
    {2,     -2, wallSize[0][2]/2},
    {-2,    6, wallSize[0][2]/2},
    {-2,    2, wallSize[0][2]/2},
    {-2,    -2, wallSize[0][2]/2},
    {4,     2, wallSize[0][2]/2},
    {-4,    2, wallSize[0][2]/2},
};

float objSize[OBJ_NUM][3] = {
    {0.1, 0.1, 0.1},
    {0.1, 0.2, 0.2},
    {0.3, 0.4, 1},
    {0.4, 0.4, 1},
    {-1, 0.3, 0.4},
};

float objPos[OBJ_NUM][3] = {
    {1.5, -0.6, objSize[0][2]/2},
    {1.7, -0.8, objSize[1][2]/2},
    {3.5, 2.6, objSize[2][2]/2},
    {3.7, -0.6, objSize[3][2]/2},
    {1.5, 0.6, objSize[4][1]/2}
};

void makeEnvironment(){
    conum = 0;
    coob[conum] = ground;
    conum++;
    for(int i=0; i<WALL_NUM; i++){
        wall[i] = dCreateBox(space, wallSize[i][0], wallSize[i][1], wallSize[i][2]);
        dGeomSetPosition(wall[i], wallPos[i][0], wallPos[i][1], wallPos[i][2]);
        
        coob[conum] = wall[i];
        conum++;
    }
    for(int i=0; i<OBJ_NUM; i++){
        if(objSize[i][0] > 0){
            object[i] = dCreateBox(space, objSize[i][0], objSize[i][1], objSize[i][2]);
            dGeomSetPosition(object[i], objPos[i][0], objPos[i][1], objPos[i][2]);
        }else{
            object[i]=dCreateSphere(space, objSize[i][1]);
            dGeomSetPosition(object[i], objPos[i][0], objPos[i][1], objPos[i][2]);
        }
        coob[conum] = object[i];
        conum++;
    }
}
void drawEnvironment(){
    const dReal *pos, *R;
    dVector3 sides;
    
    for(int i=0; i<WALL_NUM; i++){
        dGeomBoxGetLengths(wall[i],sides);
        dsSetColorAlpha(1, 1, 1.0, 1);
        pos = dGeomGetPosition(wall[i]);
        R = dGeomGetRotation(wall[i]);
        dsDrawBox(pos, R, sides);
    }
    for(int i=0; i<OBJ_NUM; i++){
        pos = dGeomGetPosition(object[i]);
        R = dGeomGetRotation(object[i]);
        if(objSize[i][0]>0){
            dGeomBoxGetLengths(object[i],sides);
            dsSetColorAlpha(1, 1, 0.0, 1);
            sides[0] = objSize[i][0];
            sides[1] = objSize[i][1];
            sides[2] = objSize[i][2];
            dsDrawBox(pos, R, sides);
        }else{
            dsSetColorAlpha(1, 0, 1.0, 1);
            dsDrawCapsule(pos, R, 0.0001, objSize[i][1]);
        }
    }
}
