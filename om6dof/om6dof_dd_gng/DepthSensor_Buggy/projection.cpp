//
//  projection.cpp
//  SimplePP-1
//
//  Created by Azhar Aulia Saputra on 4/28/16.
//  Copyright © 2016 Azhar Aulia Saputra. All rights reserved.
//

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#ifdef _WIN32
#include <windows.h>
#endif
#ifdef __APPLE__
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#else
#include <GL/glut.h>
#include <GL/gl.h>
#endif
#include "main.h"

#include "projection.h"

//#define CENTER (STARTMAP + (SCALE * sizeMap)/2)
#define CENTER 400

double dataTry[3] = {10,10,10};

double **MatrixMultiplication(double m1[3][3], double m2[3][3], int c){
    int i,j,k,l;
    double sum;
    double **out = (double **) malloc(sizeof (double *) * c);
    for (int i = 0; i < c; i++) {
        out[i] = (double *) malloc(sizeof (double) * c);
    }
    
    for(i=0; i<c; i++){
        for(j=0; j<c; j++){
            sum = 0;
            for(k=0; k<c; k++){
                sum += m1[i][k]*m2[k][j];
            }
            out[i][j] = sum;
        }
    }
    
    return out;
}


void DrawVector(int x1, int y1, int x2, int y2){
    
    glBegin(GL_LINES);
    glColor3f(0, 0, 0);
    glVertex2s(x1,y1);
    glVertex2s(x2,y2);
    glEnd();
}

double *vectorAdd(double v1[], double v2[], int j){
    double *v = (double *) malloc(sizeof (double) * j);
    
    for(int i = 0; i<j; i++)
        v[i] = v1[i] + v2[i];
    return v;
}float *vectorAdd_f(float v1[], float v2[], int j){
    float *v = (float *) malloc(sizeof (float) * j);
    
    for(int i = 0; i<j; i++)
        v[i] = v1[i] + v2[i];
    return v;
}
int multiplicationRotation(const dReal *R, double pos0[], double pos1[])
{
    double a[3][3];
    
    for(int i=0; i<3; i++){
        a[0][i] = R[0+i];
        a[1][i] = R[4+i];
        a[2][i] = R[8+i];
    }
    int i,j;
    double sum;
    
    for(i=0;i<3;i++)
    {
        sum=0;
        for(j=0;j<3;j++)   {
            sum=sum+a[i][j]*pos0[j];
//            printf("%.2f\t%.2f\t",a[i][j],pos0[j]);
        }
        pos1[i]=sum;
    }
//     for(i=0;i<3;i++)
//     {
//         printf("%.2f\t",pos0[i]);
//         printf("%.2f\t",pos1[i]);
//         printf("\n");
//     }
    return 0;
}

int vectorFromMatrixRotation(double a[3][3], double pos0[], double pos1[])
{
//    double a[3][3];
//
//    for(int i=0; i<3; i++){
//        a[0][i] = R[0+i];
//        a[1][i] = R[4+i];
//        a[2][i] = R[8+i];
//    }
    int i,j;
    double sum;
    
    for(i=0;i<3;i++)
    {
        sum=0;
        for(j=0;j<3;j++)   {
            sum=sum+a[i][j]*pos0[j];
//            printf("%.2f\t%.2f\t",a[i][j],pos0[j]);
        }
        pos1[i]=sum;
    }
//     for(i=0;i<3;i++)
//     {
//         printf("%.2f\t",pos0[i]);
//         printf("%.2f\t",pos1[i]);
//         printf("\n");
//     }
    return 0;
}
double V_out[3];
double *vectorSubtraction(double *v1, double *v2, int j){
    double *v = (double *) malloc(sizeof (double) * j);
    
    for(int i = 0; i<j; i++)
        v[i] = v1[i] - v2[i];
    return v;
}
double *vectorSubtractionD(double v1[], double v2[], int j){
//    double *v = (double *) malloc(sizeof (double) * j);
    
    for(int i = 0; i<j; i++)
        V_out[i] = v1[i] - v2[i];
    return V_out;
}
float *vectorSubtraction_f(float *v1, float *v2, int j){
    float *v = (float *) malloc(sizeof (float) * j);
    
    for(int i = 0; i<j; i++)
        v[i] = v1[i] - v2[i];
    return v;
}double *vectorProduct(double v1[], double v2[], int j){
    double *v = (double *) malloc(sizeof (double) * j);
    
    for(int i = 0; i<j; i++)
        v[i] = v1[i] * v2[i];
    return v;
}double *vectorScale(double a, double v2[], int j){
    double *v = (double *) malloc(sizeof (double) * j);
    
    for(int i = 0; i<j; i++){
        v[i] = a * v2[i];
    }
    return v;

}

double **matrixScale(double a, double m[3][3], int c){
    double **out = (double **) malloc(sizeof (double *) * c);
    for (int i = 0; i < c; i++) {
        out[i] = (double *) malloc(sizeof (double) * c);
    }
    for(int i=0; i < c; i++){
        for(int j=0; j < c; j++){
            out[i][j] = a * m[i][j];
        }
    }
    return out;

}


void vectorScaleD(double a, double v2[], int j, double v[]){
//    double *v = (double *) malloc(sizeof (double) * j);
    
    for(int i = 0; i<j; i++){
        v[i] = a * v2[i];
    }
//    return v;
}float *vectorScale_f(float a, float v2[], int j){
    float *v = (float *) malloc(sizeof (float) * j);
    
    for(int i = 0; i<j; i++){
        v[i] = a * v2[i];
    }
    return v;
}

double cosValue(double v1[], double v2[], int p){
    double a, b, out;
    a = dotProductPar(v1, v2, p);
    b = vectorValue2(v1, p) * vectorValue2(v2, p);
    
    out = a/b;
    
    return out;
}
double dotProductPar(double v1[], double v2[], int p){
    double v=0;
    for(int i=0; i < p; i++){
        v += v1[i]*v2[i];
    }
    return v;
}double *redProductPar(double v1[], double v2[], int p){
    double *v = (double *) malloc(sizeof (double) * 3);
    for(int i=0; i < p; i++){
        v[i] = v1[i] - v2[i];
    }
    return v;
}double *addProductPar(double v1[], double v2[], int p){
    double *v = (double *) malloc(sizeof (double) * 3);
    for(int i=0; i < p; i++){
        v[i] = v1[i] + v2[i];
    }
    return v;
}double dotProduct(double v1[], double v2[]){
    double v;
    v = v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
    return v;
}

double *crossProductD(double *v1, double *v2){
    double *v = (double *) malloc(sizeof (double) * 3);
    v[0] = v1[1]*v2[2] - v2[1]*v1[2];
    v[1] = -(v1[0]*v2[2] - v2[0]*v1[2]);
    v[2] = v1[0]*v2[1] - v2[0]*v1[1];
    return v;
}void crossProductD_(double *v1, double *v2, double *v){
//    double *v = (double *) malloc(sizeof (double) * 3);
    v[0] = v1[1]*v2[2] - v2[1]*v1[2];
    v[1] = -(v1[0]*v2[2] - v2[0]*v1[2]);
    v[2] = v1[0]*v2[1] - v2[0]*v1[1];
}
float *crossProduct(float *v1, float *v2){
    float *v = (float *) malloc(sizeof (float) * 3);
    v[0] = v1[1]*v2[2] - v2[1]*v1[2];
    v[1] = -(v1[0]*v2[2] - v2[0]*v1[2]);
    v[2] = v1[0]*v2[1] - v2[0]*v1[1];
    return v;
}
double *Angle3DVector(double *x1, double *x2, double *y1, double *y2, double *z1, double *z2){
    double *v = (double *) malloc(sizeof (double) * 3);
    double *v1 = (double *) malloc(sizeof (double) * 3);
    double *v2 = (double *) malloc(sizeof (double) * 3);
    double **dv1=(double **) malloc(3*sizeof(double *));
    for(int i=0;i<3;i++)
        dv1[i]=(double *) malloc(3*sizeof(double));
    double **dv2=(double **) malloc(3*sizeof(double *));
    for(int i=0;i<3;i++)
        dv2[i]=(double *) malloc(3*sizeof(double));
    
    dv1[0] = vectorUnit(x1);
    dv1[1] = vectorUnit(y1);
    dv1[2] = vectorUnit(z1);
    dv2[0] = vectorUnit(x2);
    dv2[1] = vectorUnit(y2);
    dv2[2] = vectorUnit(z2);
    
    v1[0] = atan2(dv1[0][2], dv1[0][1]);
    v1[1] = atan2(dv1[1][1], dv1[1][0]);
    v1[2] = atan2(dv1[2][2], dv1[2][0]);
    
    v2[0] = atan2(dv2[0][2], dv2[0][1]);
    v2[1] = atan2(dv2[1][1], dv2[1][0]);
    v2[2] = atan2(dv2[2][2], dv2[2][0]);
    
    for(int i=0;i<3;i++){
        v[i] = v2[i] - v1[i];
        if(v[i] > M_PI) v[i] -= 2*M_PI;
        else if(v[i] < -M_PI) v[i] += 2*M_PI;
    }
    printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n", dv1[2][0], dv1[2][1], dv1[2][2], dv2[2][0], dv2[2][1], dv2[2][2]);
    printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n", v1[0], v2[0], v1[1], v2[1], v1[2], v2[2]);
    return v;
}
double vectorValue(double v1[]){
    double v;
    v = sqrt(v1[0]*v1[0] + v1[1]*v1[1] + v1[2]*v1[2]);
    return v;
}float vectorValue_f(float v1[]){
    float v;
    v = sqrt(v1[0]*v1[0] + v1[1]*v1[1] + v1[2]*v1[2]);
    return v;
}double vectorValue2(double v1[], int p){
    double v, a = 0;
    for(int i=0; i < p; i++){
        a += v1[i]*v1[i];
    }
    v = sqrt(a);
    return v;
}
//double *v = (double *) malloc(sizeof (double) * 3);
double *vectorUnit(double v1[]){
    double a;
    double *v = (double *) malloc(sizeof (double) * 3);
//    static double v[3] = {0,0,0};
    a = vectorValue(v1);
    v[0] = v1[0]/a;
    v[1] = v1[1]/a;
    v[2] = v1[2]/a;
    return v;
}void vectorUnitD(double v1[], double v[]){
    double a;
//    double *v = (double *) malloc(sizeof (double) * 3);
//    static double v[3] = {0,0,0};
    a = vectorValue(v1);
    v[0] = v1[0]/a;
    v[1] = v1[1]/a;
    v[2] = v1[2]/a;
//    return v;
}
float *vectorUnit_f(float v1[]){
    float *v = (float *) malloc(sizeof (float) * 3);
    float a;
    a = vectorValue_f(v1);
    v[0] = v1[0]/a;
    v[1] = v1[1]/a;
    v[2] = v1[2]/a;
    
    return v;
}

float *vectorRed(float v1[], float v2[]){
    float *v = (float *) malloc(sizeof (float) * 3);
    v[0] = v1[0] - v2[0];
    v[1] = v1[1] - v2[1];
    v[2] = v1[2] - v2[2];
    return v;
}

double *location(double *Neuron0, double *Neuron1, double b, double c){
    double *pos1 = (double *) malloc(sizeof (double) * 3);
    pos1[0] = Neuron0[0] + (Neuron1[0] - Neuron0[0]) * (c/b);
    pos1[1] = Neuron0[1] + (Neuron1[1] - Neuron0[1]) * (c/b);
    pos1[2] = Neuron0[2] + (Neuron1[2] - Neuron0[2]) * (c/b);
    
    return pos1;
}
double *rotation_x(double a, double *pos){
    double *pos1 = (double *) malloc(sizeof (double) * 3);
    
    pos1[0] = pos[0];
    pos1[1] = pos[1] * cos(a) - pos[2] * sin(a);
    pos1[2] = pos[1] * sin(a) + pos[2] * cos(a);
    
    return pos1;
}double *rotation_y(double a, double *pos){
    double *pos1 = (double *) malloc(sizeof (double) * 3);
    
    pos1[0] = pos[2] * sin(a) + pos[0] * cos(a);
    pos1[1] = pos[1];
    pos1[2] = pos[2] * cos(a) - pos[0] * sin(a);
    
    return pos1;
}double *rotation_z(double a, double *pos){
    double *pos1 = (double *) malloc(sizeof (double) * 3);
    
    pos1[0] = pos[0] * cos(a) - pos[1] * sin(a);
    pos1[1] = pos[0] * sin(a) + pos[1] * cos(a);
    pos1[2] = pos[2];
    
    return pos1;
}
double *outputMatrixRotation(double *pos, double *pos0, double ax,double ay,double az){
    double *pos1 = (double *) malloc(sizeof (double) * 3);
    pos1[0] = pos[0] - pos0[0];
    pos1[1] = pos[1] - pos0[1];
    pos1[2] = pos[2] - pos0[2];
    
    pos1 = rotation_x(ax, pos1);
    pos1 = rotation_y(ay, pos1);
    pos1 = rotation_z(az, pos1);
    
    pos1[0] = pos1[0] + pos0[0];
    pos1[1] = pos1[1] + pos0[1];
    pos1[2] = pos1[2] + pos0[2];
    
    //    printf("%.3f\t%.3f\t%.3f\n",pos1[0], pos1[1], pos1[2]);
    //    DrawVector(pos1[0]*10 + 1000, pos1[1]*10 + 200 , 0 + 1000,0 + 200);
    return pos1;
}

double *Rotation3D(double *R, double *pos0){
    double *pos1 = (double *) malloc(sizeof (double) * 3);
    
    pos1[0] = pos0[0]*R[0] + pos0[1]*R[1] + pos0[2]*R[2];
    pos1[1] = pos0[0]*R[4] + pos0[1]*R[5] + pos0[2]*R[6];
    pos1[2] = pos0[0]*R[8] + pos0[1]*R[9] + pos0[2]*R[10];

    return pos1;
}
double *Rotation3D_2(double *Theta, double *pos0){
    double *pos1 = (double *) malloc(sizeof (double) * 3);
    double *pos2 = (double *) malloc(sizeof (double) * 3);
    double *pos3 = (double *) malloc(sizeof (double) * 3);
    
//    printf("%.3f\t%.3f\t%.3f\t", Theta[0], Theta[1], Theta[2]);
//    printf("%.3f\t%.3f\t%.3f\n", pos0[0], pos0[1], pos0[2]);
    
//    Theta[1] = 0;
//    Theta[2] = 0;
    
    pos1[0] = pos0[0];
    pos1[1] = pos0[1]*cos(Theta[0]) - pos0[2]*sin(Theta[0]);
    pos1[2] = pos0[1]*sin(Theta[0]) + pos0[2]*cos(Theta[0]);
    
    pos2[0] = pos1[0]*cos(Theta[1]) + pos1[2]*sin(Theta[1]);
    pos2[1] = pos1[1];
    pos2[2] = -pos1[0]*sin(Theta[1]) + pos1[2]*cos(Theta[1]);

    pos3[0] = pos2[0]*cos(Theta[2]) - pos2[1]*sin(Theta[2]);
    pos3[1] = pos2[0]*sin(Theta[2]) + pos2[1]*cos(Theta[2]);
    pos3[2] = pos2[2];

    
    return pos3;
}

double sum = 0, out;
double norm(double V[], int j){
    double sum = 0;
    for(int i = 0; i < j; i++){
        sum += V[i]*V[i];
    }
    double out = sqrt(sum);
    return out;
}
float norm_f(float V[], int j){
    float sum = 0, out;
    
    for(int i = 0; i < j; i++){
        sum += V[i]*V[i];
    }
    out = sqrt(sum);
    return out;
}
float *centerPointTriangle(double v1[], double v2[], double v3[]){
    float *v = (float *) malloc(sizeof (float) * 3);
    
    v[0] = (v1[0] + v2[0] + v3[0])/3;
    v[1] = (v1[1] + v2[1] + v3[1])/3;
    v[2] = (v1[2] + v2[2] + v3[2])/3;
    
    return v;
}

