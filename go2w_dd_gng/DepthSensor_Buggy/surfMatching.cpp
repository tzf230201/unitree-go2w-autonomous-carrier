//
//  surfMatching.cpp
//  DepthSensor_Buggy
//
//  Created by Azhar Aulia Saputra on 2020/08/25.
//  Copyright © 2020 Azhar Aulia Saputra. All rights reserved.
//

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
#include <stdio.h>
#include <string.h>
#include <vector>
#include <math.h>
#include "extDisplay.hpp"
#include "main.h"
#include "projection.h"
#include "surfMatching.hpp"
#include "gng.hpp"
#include "mesh.hpp"
#include "rnd.h"


FILE *MapData;
void printDataMap(struct gng *net){
    if((MapData = fopen("/Users/azhar/Library/Mobile Documents/com~apple~CloudDocs/Research/SLAM/outputMap.txt","w"))==NULL)
    {
        fprintf(stderr, "Cannot open output file!\n");
        exit(-1);
    }
    fprintf(MapData, "number of map node = %d\n", net->n_mapData[0]);
    for (int i=0; i<net->n_mapData[0]; i++){
        fprintf(MapData, "m");
        for (int k=0; k<8; k++) {
            fprintf(MapData, " %f",  net->mapData[0][i][k]);
        }
        fprintf(MapData, "\n");
    }
    fclose(MapData);
    
}
void RotMatrixToEulerAngles(double R[3][3], double Rot[3])
{
    double x,y,z;
    
    double sy = sqrt(R[0][0] * R[0][0] + R[1][0] * R[1][0] );

    bool singular = sy < 1e-6; // If

//    if (!singular)
//    {
//        x = atan2(R[2][1] , R[2][2] );
//        y = atan2(-R[2][0], sy);
//        z = atan2(R[1][0], R[0][0]);
//    }
    if(R[2][0] == 1.0)
    {
//        x = atan2(-R[1][2] , R[1][1]);
        x = atan2(-R[0][1] , -R[0][2]);
        y = atan2(-R[2][0], sy);
        z = 0;
    }
    else if(R[2][0] == -1.0)
    {
//        x = atan2(-R[1][2] , R[1][1]);
        
        x = atan2(R[0][1] , R[0][2]);
        y = atan2(-R[2][0], sy);
        z = 0;
    }
    else{
        x = atan2(R[2][1] , R[2][2] );
        y = atan2(-R[2][0], sy);
        z = atan2(R[1][0], R[0][0]);
    }
    y = -asin(R[2][0]);
//    return Vec3f(x, y, z);
    Rot[0] = x;// * 180/M_PI;
    Rot[1] = y;// * 180/M_PI;
    Rot[2] = z;// * 180/M_PI;
    
}
void EulerToMatrix(
            double roll,     //Yaw   angle (radians)
            double pitch,   //Pitch angle (radians)
            double yaw, double A[3][3] )   //Roll  angle (radians)
{
        //Precompute sines and cosines of Euler angles
        double su = sin(roll);
        double cu = cos(roll);
        double sv = sin(pitch);
        double cv = cos(pitch);
        double sw = sin(yaw);
        double cw = cos(yaw);
        
        A[0][0] = cv*cw;
        A[0][1] = su*sv*cw - cu*sw;
        A[0][2] = su*sw + cu*sv*cw;
        A[1][0] = cv*sw;
        A[1][1] = cu*cw + su*sv*sw;
        A[1][2] = cu*sv*sw - su*cw;
        A[2][0] = -sv;
        A[2][1] = su*cv;
        A[2][2] = cu*cv;
}
int stepMatch = 0;
double SLAMpos[3] = {0,0,0};
double SLAMrot[3] = {0,0,0};
double dSLAMrot[3] = {0,0,0};

int D[GNGN];
int nD = 0;
int E[GNGN];
int nE = 0;

void generateAssignedNode(struct gng *net){
    int a = 1;
    for (int i=0; i<net->n_mapData[1]; i+=a) {
        a = 1 + (int)(1 * rnd());
        if(net->mapData[1][i][6] == 1 || net->mapData[1][i][6] == 4 || net->mapData[1][i][6] == 5 || net->mapData[1][i][6] == 7){
            D[nD] = i;
            nD ++;
        }
    }
}
void findNearestOriginNode(struct gng *net){
    int i = 0, j = 0;
    double n1[3], n2[3], v1[3], v2[3];
    for (i = 0; i < nD; i++) {
        for (int k=0; k<3; k++) {
            n1[k] = net->mapData[1][D[i]][k];
            v1[k] = net->mapData[1][D[i]][k+3];
        }
        E[i] = -1;
        double min = 1000;
        for (j = 0; j < net->n_mapData[0]; j++) {
            if(net->mapData[0][j][6] >= 4){
                for (int k=0; k<3; k++) {
                    n2[k] = net->mapData[0][j][k];
                }
                double a = norm(vectorSubtraction(n1, n2, 3), 3);
                if(a < min && a < 0.3){
                    for (int k=0; k<3; k++) {
                        v2[k] = net->mapData[0][j][k+3];
                    }
                    double b = norm(vectorAdd(v1, v2, 3), 3);
                    
                    if(b > 0.088){
                        min = a;
                        E[i] = j;
                    }
                }
            }
        }
    }
}
double vectorMatching(struct gng *net){
    int i,j,k;
    double n1[3], n2[3], v1[3], v2[3];
    double EuAngle[3] = {0,0,0};
    double deltaNodeMove[3] = {0,0,0};
    double dataBuff[1000][9];
     double cE = 0;
    double maxError = 0, error = 0;
    double maxErrorAng = 0, errorAng = 0;
    double maxEffect = 0;
    for (i = 0; i < nD; i++) {
        if(E[i] >= 0){
            maxEffect += net->mapData[1][D[i]][7];
        }
    }
    for (i = 0; i < nD; i++) {
        if(E[i] >= 0){
            for (k=0; k<3; k++) {
                n1[k] = net->mapData[1][D[i]][k];
                v1[k] = net->mapData[1][D[i]][k+3];
                n2[k] = net->mapData[0][E[i]][k];
                v2[k] = net->mapData[0][E[i]][k+3];
            }
            double *a = vectorSubtraction(n2, n1, 3);
            double *n = vectorUnit(v1);
            double a1 = dotProduct(a, n);
            double *A1 = vectorScale(a1, n, 3);
            for (k=0; k<3; k++) {
                deltaNodeMove[k] += A1[k]  * net->mapData[1][D[i]][7];
                if(A1[k] > 10 || A1[k] < -10){
                    printf("error");
                }
            }
            error = maxError + norm(A1, 3);
//            if(error > maxError)
            maxError = error;
            
            free(a);
            free(n);
            free(A1);
            
            cE += 1.0;
            
            
            double ang[3];
            vectorToEuler(v1, v2, ang);
            if(( ang[0] != ang[0])||( ang[1] != ang[1])||( ang[2] != ang[2])){
                printf("");
            }
            
            for (k=0; k<3; k++)
                EuAngle[k] += ang[k] * net->mapData[1][D[i]][7];
            if(ang[2] > 100 || ang[2] < -100){
                printf("error %f\n", ang[2]);
            }
//            for (k=0; k<3; k++){
//                dataBuff[i][k] = ang[k];
//                dataBuff[i][k+3] = v1[k];
//                dataBuff[i][k+6] = v2[k];
//            }
            
            
//            printf("%f\t%f\t%f\n", EuAngle[0], EuAngle[1], EuAngle[2]);
            
        }
    }
    
    double F[3][3];
    double v[3];
    double a[3];
    if(cE < 1){
        printf("Error\n");
    }else{
        for (int k=0; k<3; k++) {
            if(EuAngle[k] > 0.001*cE || EuAngle[k] < -0.001*cE)
                a[k] = 0.3 * EuAngle[k]/maxEffect;
            else
                a[k] = 0;
            if(a[k] != a[k]){
                printf("");
            }
            if(a[k] > 100 || a[k] < -100){
                printf("error %f\n", a[k]);
            }
        }
        if((a[1] > 0.03 || a[1] < -0.03) && SLAMpos[2] > M_PI/2){
            printf("");
        }
        EulerToMatrix(a[0] ,a[1] ,a[2] , F);
            
        for(i = 0; i < net->n_mapData[1]; i++){
            for (int k=0; k<3; k++) {
                v1[k] = net->mapData[1][i][k] - SLAMpos[k];
            }
            vectorFromMatrixRotation(F, v1, v2);
            for (int k=0; k<3; k++) {
                net->mapData[1][i][k] = v2[k] + SLAMpos[k];
            }
    //
            for (int k=0; k<3; k++) {
                v1[k] = net->mapData[1][i][k+3];
            }
            vectorFromMatrixRotation(F, v1, v2);
            for (int k=0; k<3; k++) {
                net->mapData[1][i][k+3] = v2[k];
            }
        }
//        printf("num %.0f rot %f\t%f\t%f\n", cE, a[0], a[1], a[2]);
        
        double A = 100 * (maxError/cE);
        for(i = 0; i < net->n_mapData[1]; i++){
            for (int k=0; k<3; k++) {
                net->mapData[1][i][k] += 0.3 * deltaNodeMove[k]/maxEffect;
            }
        }

        for (int k=0; k<3; k++) {
            SLAMpos[k] += 0.3 * deltaNodeMove[k]/maxEffect;
            if(deltaNodeMove[k]/maxEffect > 1000){
                printf("error");
            }
        }
    //    printf("pos %f\t%f\t%f\n", SLAMpos[0], SLAMpos[1], SLAMpos[2]);
        for (int k=0; k<3; k++) {
            SLAMrot[k] += a[k];
        }
    }
    if(SLAMpos[2] > 0.75){
        printf("error");
    }
    SLAMrot[0] = 0;
    SLAMrot[1] = 0;
    return maxError/cE;
//    printf("%.3f\t",SLAMpos[0]);
}
void add_n_changeOriginNode(struct gng *net){
    int i = 0, j = 0, k = 0;
    double n1[3], n2[3], v1[3], v2[3];
    int add[net->n_mapData[1]];
    int n_map0 = net->n_mapData[0];
    int count = n_map0;
    for (i = 0; i < net->n_mapData[1]; i++) {
        add[i] = 0;
        for (int k=0; k<3; k++) {
            n1[k] = net->mapData[1][i][k];
            v1[k] = net->mapData[1][i][k+3];
        }
        E[i] = -1;
        double min = 1000;
        double min2 = 1000;
        double near = 0;
        int con[15];
        int ncon = 0;
        for (j = 0; j < net->n_mapData[0]; j++) {
            for (int k=0; k<3; k++) {
                n2[k] = net->mapData[0][j][k];
            }
            double a = norm(vectorSubtraction(n1, n2, 3), 3); // distance between GNG data and map data
            if(a < min && a < 0.3){
                for (int k=0; k<3; k++) {
                    v2[k] = net->mapData[0][j][k+3];
                }
                double b = norm(vectorAdd(v1, v2, 3), 3);

                if(b > 0.09){
                    min = a;
                    E[i] = j;
                }
            }
            if(a < 0.6 && ncon < 15){ // connect new GNG data and old map data;
                con[ncon] = j;
                ncon++;
            }
            near = min;
            if(a < min2){
                min2 = a;
            }
            if (net->mapData[1][i][6] >= 4  && near < 0.2) // new GNG data cannot be added into the map data ;
                break;
        }

        // find the edge to the nearest map nodes
        if((net->mapData[1][i][6] >= 4 && net->mapData[1][i][6] < 6  && near > 0.2 ) ){ // connect new nodes with old nodes
            add[i] = 1;
            for(j = 0; j < ncon; j++){
                net->mapEdge[0][con[j]][count] = 1;
                net->mapEdge[0][count][con[j]] = 1;
            }
            count++;
        }
    }
    if(net->n_mapData[0] > 10000 || net->n_mapData[0] <= 0){
        printf("error");
    }
    count = n_map0;
    for(int i=0; i < net->n_mapData[1]; i++){
        if(add[i] == 1){
            for (k=0; k<8; k++) {
                net->mapData[0][count][k] = net->mapData[1][i][k];
            }
            // find the edge from the current GNG
            int count2 = n_map0;
            for(int j=0; j < net->n_mapData[1]; j++){
                if(add[j] == 1){
                    if(net->mapEdge[1][i][j] == 1){
                        net->mapEdge[0][count][count2] = 1;
                        net->mapEdge[0][count2][count] = 1;
                    }else{
                        net->mapEdge[0][count][count2] = 0;
                        net->mapEdge[0][count2][count] = 0;
                    }
                    count2++;
                }
            }
            count ++;
        }
    }
    
    net->n_mapData[0] = count;
    
    triangle_search_with_range(net, 0, net->n_mapData[0]);
    sphereOfTriangle(net);
    reconstructEdge(net, 0, net->n_mapData[0]);
    search_connection_num(net);
    
    net->n_mapData[1] = 0;
}

void surfaceMatching(struct gng *net){
    if(stepMatch == 0){
        nD = 0; nE = 0;
        stepMatch = 1;
    }else if(stepMatch == 1){
        generateAssignedNode(net);
//        for (int i=0; i<nD; i++) {
//            double n[3];
//            for (int k=0; k<3; k++) {
//                n[k] = net->mapData[1][D[i]][k];
//            }
//            glColor3f(0.6, 0.2, 0.6);
//            drawPoint(n, 30);
//        }
        stepMatch = 2;
    }else if(stepMatch == 2){
//        static int flag = 0;
//        if(flag == 0){
            findNearestOriginNode(net);
//            flag = 1;
//        }
//        for (int i=0; i<nD; i++) {
//            double n[3];
//            for (int k=0; k<3; k++) {
//                n[k] = net->mapData[1][D[i]][k];
//            }
//            glColor3f(0.6, 0.2, 0.6);
//            drawPoint(n, 20);
//            if(E[i] >= 0){
//                for (int k=0; k<3; k++) {
//                    n[k] = net->mapData[0][E[i]][k];
//                }
//                glColor3f(0.0, 0.8, 0.6);
//                drawPoint(n, 20);
//            }
//        }
        
        stepMatch = 3;
    }else if(stepMatch == 3){
        double maxError = 100;
        int count =0 ;
        while(maxError > 0.003 && count < 200){
            maxError = vectorMatching(net);
            count++;
                
//            else
//                stepMatch = 0;
        }
        stepMatch = 4;
        
        for (int i=0; i<nD; i++) {
            double n[3];
            for (int k=0; k<3; k++) {
                n[k] = net->mapData[1][D[i]][k];
            }
            glColor3f(0.6, 0.2, 0.6);
            drawPoint(n, 10);
            if(E[i] >= 0){
                for (int k=0; k<3; k++) {
                    n[k] = net->mapData[0][E[i]][k];
                }
                glColor3f(0.0, 0.8, 0.6);
                drawPoint(n, 10);
            }
        }
//        nD = 0; nE = 0;
//        generateAssignedNode(net);
//        findNearestOriginNode(net);
//        for (int i = 0; i < 200; i++)
//            vectorMatching(net);
        
    }else if(stepMatch == 4){
        add_n_changeOriginNode(net);
        stepMatch = 5;
    }
}


void vectorToEuler(double v1[3], double v2[3], double out[3]){
    double R[3][3];
//    double F[3][3];
//    double v[3];
//    double v1[3] = {0, 0, 1};
//    double v2[3];
    
//    EulerToMatrix(0, 0.1, 0.1, F);
    
//    RotMatrixToEulerAngles(F, v);
    
//    vectorFromMatrixRotation(F, v1, v2);
    
    
    double *a = vectorUnit(v1);
    double *b = vectorUnit(v2);
//    printf("a = %f\t%f\t%f\n", a[0], a[1], a[2]);
//    printf("b = %f\t%f\t%f\n", b[0], b[1], b[2]);
    double th = acos(dotProduct(a, b));
    
    if(th == 0){
        out[0] = 0;out[1] = 0;out[2] = 0;
    }else{
        double *u1 = crossProductD(a, b);
        double *u = vectorUnit(u1);
//        printf("u1 = %f\t%f\t%f\n", u1[0], u1[1], u1[2]);
//        printf("u = %f\t%f\t%f\n", u[0], u[1], u[2]);
        
        double K[3][3] = {
            {0, -u[2], u[1]},
            {u[2], 0, -u[0]},
            {-u[1], u[0], 0}};
        double I[3][3] = {
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1}};
        double sth = sin(th);
        double A[3][3] = {
            {sth*K[0][0], sth*K[0][1], sth*K[0][2]},
            {sth*K[1][0], sth*K[1][1], sth*K[1][2]},
            {sth*K[2][0], sth*K[2][1], sth*K[2][2]},
        };
        double **B = MatrixMultiplication(K, K, 3);
        double cth = 1-cos(th);
        double C[3][3] = {
            {cth*B[0][0], cth*B[0][1], cth*B[0][2]},
            {cth*B[1][0], cth*B[1][1], cth*B[1][2]},
            {cth*B[2][0], cth*B[2][1], cth*B[2][2]},
        };
        for(int i=0; i<3; i++){
            for(int j=0; j<3; j++){
                R[i][j] = I[i][j] + A[i][j] + C[i][j];
            }
        }
        RotMatrixToEulerAngles(R, out);
        
        free(u1);
        free(u);
    }
    free(a);
    free(b);
//    printf("out = %f\t%f\t%f\n", out[0], out[1], out[2]);
//    return v;
}
double vectorToEulerTest(){
    double R[3][3];
    double F[3][3];
    double out[3];
    double v[3];
    double v1[3] = {0, 1, 0};
    double v2[3];
    
    EulerToMatrix(0, 0.0, 0.2, F);
    
    RotMatrixToEulerAngles(F, v);
    
    vectorFromMatrixRotation(F, v1, v2);
    
    
    double *a = vectorUnit(v1);
    double *b = vectorUnit(v2);
//    printf("a = %f\t%f\t%f\n", a[0], a[1], a[2]);
//    printf("b = %f\t%f\t%f\n", b[0], b[1], b[2]);
    double th = acos(dotProduct(a, b));
    
    if(th == 0){
        out[0] = 0;out[1] = 0;out[2] = 0;
    }else{
        double *u1 = crossProductD(a, b);
        double *u = vectorUnit(u1);
//        printf("u1 = %f\t%f\t%f\n", u1[0], u1[1], u1[2]);
//        printf("u = %f\t%f\t%f\n", u[0], u[1], u[2]);
        
        double K[3][3] = {
            {0, -u[2], u[1]},
            {u[2], 0, -u[0]},
            {-u[1], u[0], 0}};
        double I[3][3] = {
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1}};
        double sth = sin(th);
        double A[3][3] = {
            {sth*K[0][0], sth*K[0][1], sth*K[0][2]},
            {sth*K[1][0], sth*K[1][1], sth*K[1][2]},
            {sth*K[2][0], sth*K[2][1], sth*K[2][2]},
        };
        double **B = MatrixMultiplication(K, K, 3);
        double cth = 1-cos(th);
        double C[3][3] = {
            {cth*B[0][0], cth*B[0][1], cth*B[0][2]},
            {cth*B[1][0], cth*B[1][1], cth*B[1][2]},
            {cth*B[2][0], cth*B[2][1], cth*B[2][2]},
        };
        for(int i=0; i<3; i++){
            for(int j=0; j<3; j++){
                R[i][j] = I[i][j] + A[i][j] + C[i][j];
            }
        }
        RotMatrixToEulerAngles(R, out);
    }
//    printf("out = %f\t%f\t%f\n", out[0], out[1], out[2]);
    return 0;
}
