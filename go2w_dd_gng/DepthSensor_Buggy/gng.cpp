/*
 *  gng.c
 *  Claster
 *
 *  Created by Naoyuki Kubota on 12/05/18.
 *  Copyright 2012 首都大学東京. All rights reserved.
 *
 */


#include "gng.hpp"

//extern "C"{
#include "rnd.h"
#include "malloc.h"
#include "projection.h"
#include "main.h"
#include "extDisplay.hpp"
#include "surfMatching.hpp"
//#include "showOpenGL.hpp"
//};
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <iostream>
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



struct gng *ocnet = NULL;
struct gng *intnet = NULL;

struct gng *init_gng()
{
    int i,j,k;
    struct gng *net=NULL;
    if(net == NULL)
        net = (struct gng *)malloc(sizeof (struct gng));
    
    for (j = 0; j < DIM; j++) {
        net->weight[j] = 1.0;
    }
    
    for(i=0;i<GNGN;i++){
        for(j=0;j<3;j++)
            for(k=0;k<3;k++)
                net->susLabelSize[i][j][k] = 0;
        net->strength[i] = 1;
        net->normAge[i] = 0;
        for(j=0;j<DIM;j++){
            net->node[i][j] = 1 * rnd();
            net->prenode[i][j] = net->node[i][j];
            net->delta[i][j] = 0;

            net->normTriangle[i][j] = 0;
        }
        for(j=0;j<5;j++){
            net->triangle[i][j] = 0;
            net->normVect[i][j] = 0;
//            net->doubleCon[i][j] = 0;
        }
        
        
        net->gng_err[i] = 0;
        net->gng_u[i] = 0.0;
        net->movedis[i] = 0.0;
        net->safedeg[i] = 0.0;
        net->edge_dimension[i] = 0;
        
        
        
        for(j=0;j<GNGN;j++){
            net->edge[i][j] = 0;
            net->edgeTriangle[i][j] = 0;
            net->age[i][j] = 0;
        }
    }
    
    for(i=0;i<2;i++)
        for(j=0;j<2;j++){
            if(i != j)
                net->edge[i][j] = 1;
        }
    net->node_n = 2;
    net->n_susLabel = 0;
    
    net->sigma = 20.0;
    net->cog[0] = 0.0;
    net->cog[1] = 0.0;
    net->cog[2] = 1.0;
    
    net->parent = NULL;
    net->child = NULL;
    net->layer = 0;
    net->triangle_n = 0;
    net->doubleCon_n = 0;
    net->intentional_n = 0;
    net->intNode_n = 0;
    net->square_n = 0;
    net->K = 100000;
    net->udrate = 0.001;
    
    for(int i = 0; i < 2; i++){
        net->n_mapData[i] = 0;
        for(j=0;j<GNGN;j++){
            for(int k = 0; k < 8; k++)
                net->mapData[i][j][k] = 0;
        }
    }
    return net;
}
void getGNGdata(struct gng *net, int c){
    int count = 0;
    double F[3][3];
    double v1[3], v2[3];
    EulerToMatrix(SLAMrot[0] ,SLAMrot[1] ,SLAMrot[2] , F);
    for(int j=0;j< net->node_n;j++){
        if(net->normTriangle[j][0] != 3){
            for(int k=0; k<3; k++){
                v1[k] = net->node[j][k];
            }
            vectorFromMatrixRotation(F, v1, v2);
            for(int k = 0; k < 3; k++)
                net->mapData[c][count][k] = v2[k] + SLAMpos[k];
            
            for(int k=0; k<3; k++){
                v1[k] = net->normVect[j][k];
            }
            vectorFromMatrixRotation(F, v1, v2);
            for(int k = 3; k < 6; k++)
                net->mapData[c][count][k] = v2[k-3];
            
//            for(int k = 6; k < 8; k++)
            net->mapData[c][count][6] = net->normTriangle[j][0];
            net->mapData[c][count][7] = 10 * net->normTriangle[j][1];
            
            int count2 = 0;
            for(int k=0;k< net->node_n;k++){
                if(net->normTriangle[k][0] != 3){
                    if(net->edge[k][j] == 1){
                        net->mapEdge[c][count][count2] = 1;
                        net->mapEdge[c][count2][count] = 1;
                    }else{
                        net->mapEdge[c][count][count2] = 0;
                        net->mapEdge[c][count2][count] = 0;
                    }
                    count2++;
                }
            }
            count ++;
        }
    }
    net->n_mapData[c] = count;
}
void gng_triangle_search(struct gng *net)
{
    int i,j,k;
    int ct = 0;
    for(i=0;i<net->node_n;i++){
        for(j=i+1;j<net->node_n;j++){
            for(k=j+1;k<net->node_n;k++){
                if(net->node[i][0] == 0 && net->node[i][1] == 0 && net->node[i][2] == 0) continue;
                if(net->node[j][0] == 0 && net->node[j][1] == 0 && net->node[j][2] == 0) continue;
                if(net->node[k][0] == 0 && net->node[k][1] == 0 && net->node[k][2] == 0) continue;
                if(i > GNGN || j > GNGN || k > GNGN){
                    printf("%d,%d,%d\n",i,j,k);
                    continue;
                }
                
                if((net->edge[i][j] == 1 || net->edgeTriangle[i][j] == 1) && (net->edge[i][k] == 1 || net->edgeTriangle[i][k] == 1) && (net->edge[k][j] == 1 || net->edgeTriangle[k][j] == 1) && i!=j && j !=k && k != i){
                    net->triangle[ct][0] = i;
                    net->triangle[ct][1] = j;
                    net->triangle[ct][2] = k;
                    ct++;
                }
            }
        }
    }
    net->triangle_n = ct;
}

void search_square(struct gng *net)
{
    int a,b,c,d,i,j;
    int ct = 0;
//    for(i=0;i<GNGN;i++){
//        for(j=0;j<GNGN;j++){
//            net->edgeTriangle[i][j] = 0;
//        }
//    }
    for(a=0;a<net->node_n;a++){
        for(b=0;b<net->node_n;b++){
            for(c=0;c<net->node_n;c++){
                if(net->edge[a][b] == 1 && net->edge[a][c] == 1 && net->edge[c][b] == 0 && net->edgeTriangle[c][b] == 0 && a != b && b != c && c != a){
                    for(d=0;d<net->node_n;d++){
                        if(net->edge[d][b] == 1 && net->edge[d][c] == 1 && net->edge[d][a] == 0 && net->edgeTriangle[d][a] == 0 && a != d){
                                net->edgeTriangle[d][a] = 1;
                                net->edgeTriangle[a][d] = 1;
                                ct++;
                        }
                    }
                }
            }
        }
    }
    net->square_n = ct;
}

void search_pentagon(struct gng *net)
{
    int i, j, a,b,c,d,e,o;
    int ct = 0, cr = 0, cp = 0;
    
    
    
    for(a=0;a<net->node_n;a++){
        for(b=0;b<net->node_n;b++){
            for(c=0;c<net->node_n;c++){
                if(net->edge[a][b] == 1 && net->edge[b][c] == 1 && net->edge[c][a] == 0 && net->edgeTriangle[c][a] == 0 && a != b && b != c && c != a){
                    cp = 0;
                    for(d=0;d<net->node_n;d++){
                        if(net->edge[d][c] == 1 && net->edge[d][b] == 0 && net->edge[d][a] == 0 && net->edgeTriangle[d][b] == 0 && net->edgeTriangle[d][a] == 0 && d != a && d != b && d != c){
                            cp++;
                            cr = 0;
                            for(e=0;e<net->node_n;e++){
                                if(net->edge[e][d] == 1 && net->edge[e][a] == 1 && net->edge[e][b] == 0 && net->edge[e][c] == 0 && e != a && e != b && e != c && e != d){
                                    cr++;
                                }
                            }
                            
                        }
                    }
                    
                    if(cr >= 1 && cp >= 1){
                        net->edgeTriangle[d][a] = 1;
                        net->edgeTriangle[a][d] = 1;
                        net->edgeTriangle[c][a] = 1;
                        net->edgeTriangle[a][c] = 1;
                        
                        ct++;
                    }
                }
            }
        }
    }
}

void search_hexagon(struct gng *net)
{
    int a,b,c,d,e,f,o;
    int ct = 0;
    for(a=0;a<net->node_n;a++){
        for(b=0;b<net->node_n;b++){
            for(c=0;c<net->node_n;c++){
                if(net->edge[a][b] == 1 && net->edge[a][c] == 1 && net->edge[c][b] == 0 && a != b && b != c && c != a){
                    for(d=0;d<net->node_n;d++){
                        for(e=0;e<net->node_n;e++){
                            if(net->edge[d][b] == 1 && net->edge[e][c] == 1 && net->edge[e][d] == 0 && d != e){
                                for(f=0;f<net->node_n;f++){
                                    if(net->edge[e][f] == 1 && net->edge[d][f] == 1){
                                        if(net->edge[a][f] == 0 && net->edge[b][f] == 0 && net->edge[c][f] == 0 && f != a && f != b && f != c){
                                            if(net->edge[d][a] == 0 && net->edge[d][c] == 0 && net->edge[e][b] == 0 && net->edge[e][a] == 0 && d != a && e != a){
                                                ct = 0;
                                                for(o=0;o<net->node_n;o++)
                                                    if(net->edge[o][a] == 1 && net->edge[o][b] == 1 && net->edge[o][c] == 1 && net->edge[o][d] == 1 && net->edge[o][e] == 1 && net->edge[o][f] == 1)
                                                        ct++;
                                                if(ct == 0){
                                                    net->edge[d][a] = 1;
                                                    net->edge[a][d] = 1;
                                                    net->edge[e][a] = 1;
                                                    net->edge[a][e] = 1;
                                                    net->edge[f][a] = 1;
                                                    net->edge[a][f] = 1;
                                                    
                                                    net->age[d][a] = 0;
                                                    net->age[a][d] = 0;
                                                    net->age[e][a] = 0;
                                                    net->age[a][e] = 0;
                                                    net->age[f][a] = 0;
                                                    net->age[a][f] = 0;
                                                    
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
void search_doubleConnection(struct gng *net){
    
    int i,j, a[3];
    int ct = 0;
    int c = net->triangle_n;
    for(i=0;i<net->node_n;i++){
        ct = 0;
        for(j=0;j<net->node_n;j++){
            if(net->edge[i][j] == 1 && net->edge[j][i] == 1){
                a[ct] = j;
                ct++;
                if(ct > 2) j = net->node_n;
            }
        }
        if(ct == 2){
            net->triangle[c][0] = i;
            net->triangle[c][1] = a[0];
            net->triangle[c][2] = -1;
            net->triangle[c+1][0] = i;
            net->triangle[c+1][1] = a[1];
            net->triangle[c+1][2] = -1;
            c+=2;
        }
    }
    net->doubleCon_n = c - net->triangle_n;
//    printf("num = %d, %d, %d\n",c,net->triangle_n,net->doubleCon_n);
}
double *getMaxLabel(struct gng *net, int id, double *max_old){
    double *max = (double *) malloc(sizeof (double) * 3);
    int i;
    for(i = 0; i < 3; i++){
        double val = (net->node[net->susNode[id]][i] + net->normTriangle[net->susNode[id]][1]/2);
        if(max_old[i] < val){
            max[i] = val;
        }else{
            max[i] = max_old[i];
        }
    }
    return max;
}

double *getMinLabel(struct gng *net, int id, double *min_old){
    double *min = (double *) malloc(sizeof (double) * 3);
    int i;
    for(i = 0; i < 3; i++){
        double val = (net->node[net->susNode[id]][i] - net->normTriangle[net->susNode[id]][1]/2);
        if(min_old[i] > val){
            min[i] = val;
        }else{
            min[i] = min_old[i];
        }
    }
    return min;
}

void searchConnectedSusNode(struct gng *net, int n1, bool &connected){
    int i;
    //    printf("n = %.2f\t%.2f\t%.2f\tc = %d\n", n[0], n[1], n[2], cNodeRung);
    
    for(i = 0; i < net->susNode_n; i++){
        if(net->susNodeLabel[i] == 0 && (net->edge[net->susNode[i]][net->susNode[n1]] == 1 || net->edge[net->susNode[n1]][net->susNode[i]] == 1)){
                // find next connection;
            net->susNodeLabel[i] = net->susNodeLabel[n1];
            
            double *min = getMinLabel(net, i, net->susLabelSize[net->susNodeLabel[i]-1][0]);
            double *max = getMaxLabel(net, i, net->susLabelSize[net->susNodeLabel[i]-1][1]);
            for(int k=0; k<3; k++){
                net->susLabelSize[net->susNodeLabel[i]-1][0][k] = min[k];
                net->susLabelSize[net->susNodeLabel[i]-1][1][k] = max[k];
            }
            
            searchConnectedSusNode(net, i, connected);
        }
    }
}
void expectObstacleSize(struct gng *net){
    int i;
    int curLabel = 0;
    bool connected = 0;
    for(i = 0; i < net->susNode_n; i++){
        net->susNodeLabel[i] = 0;
    }
    for(i = 0; i < net->susNode_n; i++){
        if(net->susNodeLabel[i] == 0){
            for(int k=0; k<3; k++){
                net->susLabelSize[curLabel][0][k] = 100000;
                net->susLabelSize[curLabel][1][k] = -100000;
            }
            curLabel ++;
            net->susNodeLabel[i] = curLabel;
            double *min = getMinLabel(net, i, net->susLabelSize[net->susNodeLabel[i]-1][0]);
            double *max = getMaxLabel(net, i, net->susLabelSize[net->susNodeLabel[i]-1][1]);
            for(int k=0; k<3; k++){
                net->susLabelSize[net->susNodeLabel[i]-1][0][k] = min[k];
                net->susLabelSize[net->susNodeLabel[i]-1][1][k] = max[k];
            }
            searchConnectedSusNode(net, i, connected);
        }
        
    }
    net->n_susLabel = curLabel;
}
int segmentInt[segmentNum];
//double **nn;
//double **no;

double nn[10][3];
double no[10][3];
double np[10][3];
void calc_node_normal_vector(struct gng *net, double *sP, int showArrow){
    int i,j,k;
    static int flag = 0;
//    if(flag == 0){
//        nn = (double **) malloc(sizeof (double *) * 10);
//        for (int i = 0; i < 3; i++) {
//            nn[i] = (double *) malloc(sizeof (double) * 3);
//        }
//        no = (double **) malloc(sizeof (double *) * 10);
//        for (int i = 0; i < 3; i++) {
//            no[i] = (double *) malloc(sizeof (double) * 3);
//        }
//        flag = 1;
//    }
//    showArrow = 0;
    sP[0] = 0;
    sP[1] = 0;
    sP[2] = 0;
    double vf[3] = {0,0,1}, va, vt;
    net->susNode_n = 0;
//    for(i=0; i<1; i++){
    for(i=0; i<net->node_n; i++){
        int nCon = 0;
        int lastCon = net->normTriangle[i][0];
        if(lastCon > 3) lastCon = lastCon - 4;
        net->normTriangle[i][0] = 3;
        net->normTriangle[i][1] = 0;

//        net->strength[i] = 1;
        double sumVect = 0;
        
        for (j = 0; j<net->node_n; j++) {
            if (net->edge[i][j] == 1 && net->edge[j][i] == 1) {
                double *v = vectorSubtraction(net->node[i], net->node[j], 3);
                sumVect += norm(v, 3);
                vectorUnitD(v,nn[nCon] );
                nCon++;
                free(v);
            }
        }
        if(nCon > 2){

            net->normTriangle[i][1] = sumVect/(double)(nCon);
            crossProductD_(nn[nCon-1], nn[0],no[0]);
            for(j=1; j<nCon; j++){
                crossProductD_(nn[j-1], nn[j], no[j]);
//                no[j] = crossProductD(nn[j-1], nn[j]);
            }
            
//            glColor3f(1.0f, 1.0f, 0.0f);
            double *vRef = vectorSubtraction(sP, net->node[i], 3);
            for(j=0; j<nCon; j++){
                vectorUnitD(no[j], no[j]);
                
                double scale = 0.05;
//                const clock_t begin_time = clock();
                double *aa = vectorSubtraction(vRef, no[j], 3);
                double a = norm(aa, 3);
                free(aa);
                double b = norm(no[j], 3);
                double c = norm(vRef, 3);
                double d = sqrt(b*b + c*c);
//                printf("%.8f\n",double( clock () - begin_time ) /  CLOCKS_PER_SEC);
                if(a > d)
                    scale = -0.05;
                vectorScaleD(scale, no[j], 3, np[j]);
                
//                    double *v  = vectorAdd(net->node[i], np[j], 3);
//                    glColor3f(0, 0, 1);
//                    drawLine(net->node[i], v, 2);
            }
            
            double *m = vectorAdd(np[0], np[1], 3);
            for(j=2; j<nCon; j++){
                double *l = vectorAdd(m, np[j], 3);
                for(k = 0; k < 3; k++) m[k] = l[k];
                free(l);
            }
            double *n = vectorScale(1/((double)(nCon)),m, 3);
            
            for(int k = 0; k < 3; k++){
                net->normVect[i][k] = n[k];
            }
            
            double a = norm(n, 3);
            va = n[0]*vf[0] + n[1]*vf[1] + n[2]*vf[2];
            vt = va/(norm(n, 3) * norm(vf, 3));
            double b = acos(vt);
            
            double A = 1;
            net->strength[i] = 1;
//            if(net->node[i][1] < 0.3 && net->node[i][1] > -0.3 && net->node[i][0] > 2.5){
//                A = 1;
//                net->susNode[net->susNode_n] = i;
//                net->susNode_n++;
//                net->strength[i] = 20;
//            }
            
//            net->strength[i] = 20;
            net->normTriangle[i][0] = 0;
            if(a < 0.049){
//                glColor3f(1.0f, 1.0f, 0.0f);
                net->normTriangle[i][0] = 2;
                if(A == 1){
                    net->susNode[net->susNode_n] = i;
                    net->susNode_n++;
                    net->strength[i] = 20;
                }
            }
            else if(b > M_PI/12){
//                glColor3f(1.0f, 0.0f, 0.0f);
                net->normTriangle[i][0] = 1;
//                if(A == 1){
//                    net->susNode[net->susNode_n] = i;
//                    net->susNode_n++;
//                    net->strength[i] = 20;
//                }
            }
            free(n);
            free(m);
            free(vRef);
            
            if((net->normTriangle[i][0] <= 3) && net->normTriangle[i][0] != lastCon){
                net->normAge[i] = 0;
            }
            else if(net->normTriangle[i][0] == 0 || net->normTriangle[i][0] == 1 || net->normTriangle[i][0] == 2){
                net->normAge[i] ++;
            }else{
                net->normAge[i] --;
            }
                
            if(net->normAge[i] < 0) net->normAge[i] = 0;
            else if(net->normAge[i] > 5) net->normAge[i] = 5;
            
            if(net->normTriangle[i][0] == 0 && net->normAge[i] > 4){
                net->normTriangle[i][0] = 4;
            }else if(net->normTriangle[i][0] == 1 && net->normAge[i] > 4){
                net->normTriangle[i][0] = 5;
            }else if(net->normTriangle[i][0] == 2 && net->normAge[i] > 4){
                net->normTriangle[i][0] = 6;
                
            }
        }
    }
//    free(nn);
//    free(no);
//    expectObstacleSize(net);
}


void gng_IntentionNodeTransfer(struct gng *net, struct gng *net2){
    int a=0, i, j, k;
    net2->node_n = 0;
    for(a=0; a<net->triangle_n; a++){
        if(net->triangle[a][4] == 1){
            for(i =0; i<3; i++){
                int sim = 0;
                for(j=0; j<a; j++){
                    for(k =0; k<3; k++){
                    }
                }
                if(sim == 0){
                    net2->node_n++;
                }
            }
        }
    }
}
void gng_triangulation(struct gng *net)
{
    int i, j;
    for(i=0;i<GNGN;i++){
        for(j=0;j<GNGN;j++){
            net->edgeTriangle[i][j] = 0;
        }
    }
    search_pentagon(net);
    search_square(net);
    gng_triangle_search(net);
    search_doubleConnection(net);
//    normal_vector_triangulation(net);
}

void gng_classification(struct gng *net)
{
    int i,j,k,n;
    int sflag=0;
    int flag[GNGN];
    int c=0;
    int c_n=0;
    int sum=0;
    for(i=0;i<net->node_n;i++)
        flag[i] = 0;
    
    net->cluster[c][c_n] = 0;
    flag[0] = 1;
    c_n++;
    
    while (sum < net->node_n){
        for(i=0;i<c_n;i++){
            n=net->cluster[c][i];
            for(j=0;j<net->node_n;j++){
                if(n != j && net->edge[n][j] == 1){
                    sflag = 0;
                    for(k=0;k<c_n;k++)
                        if(j == net->cluster[c][k])
                            sflag = 1;
                    
                    if(sflag == 0){
                        net->cluster[c][c_n] = j;
                        flag[j] = 1;
                        c_n++;
                    }
                }
            }
        }
        sum+=c_n;
        net->cluster_num[c] = c_n;
        c_n = 0;
        c++;
        for(i=0;i<net->node_n;i++){
            if(flag[i] == 0){
                net->cluster[c][c_n] = i;
                flag[i] = 1;
                c_n++;
                break;
            }
        }
    }
    
    net->cluster_ct = c;
}


void discount_err_gng(struct gng *net)
{
    int i;
    net->udrate = 0.001;
    for(i=0;i<net->node_n;i++){
        net->gng_err[i] -= (0.001/(net->strength[i] * net->strength[i] * net->strength[i]* net->strength[i]))*net->gng_err[i];
        
        
        net->gng_u[i] -= (net->udrate)*net->gng_u[i];
        
            
        
        if(net->gng_err[i] < 0)
            net->gng_err[i] = 0.0;
        
        if(net->gng_u[i] < 0)
            net->gng_u[i] = 0.0;
        
    }
}

int node_delete(struct gng *net, int i)
{
    int j,k,l;
    
    for(j=i;j<net->node_n;j++){
        
        net->gng_err[j] = net->gng_err[j+1];
        net->strength[j] = net->strength[j+1];
        net->gng_u[j] = net->gng_u[j+1];
        net->movedis[j] = net->movedis[j+1];
        net->wct[j] = net->wct[j+1];
        net->normAge[j] = net->normAge[j+1];
        
        
        for(l=0;l<DIM;l++){
            if(l < 3){
                net->dir[j][l] = net->dir[j+1][l];
            }
            net->node[j][l] = net->node[j+1][l];
            net->prenode[j][l] = net->prenode[j+1][l];
            net->normTriangle[j][l] = net->normTriangle[j+1][l];
        }
        for(l=0;l<5;l++){
            net->normVect[j][l] = net->normVect[j+1][l];
        }
        for(k=0;k<net->node_n;k++){
            if(k < i){
                net->age[j][k] = net->age[j+1][k];
                net->age[k][j] = net->age[k][j+1];
                net->edge[j][k] = net->edge[j+1][k];
                net->edge[k][j] = net->edge[k][j+1];
            }else{
                net->age[j][k] = net->age[j+1][k+1];
                net->edge[j][k] = net->edge[j+1][k+1];
            }
        }
    }
    
    for(k=0;k<net->node_n;k++){
        net->age[net->node_n][k] = 0;
        net->age[k][net->node_n] = 0;
        net->edge[net->node_n][k] = 0;
        net->edge[k][net->node_n] = 0;
    }
    net->normAge[net->node_n] = 0;
    
    net->node_n--;
    if (net->node_n <= 0) {
        printf("test");
    }
    return net->node_n;
}

void node_add_gng2(struct gng *net, int flag)
{
    int i,j;
    double max_err[2];
    double min_u;
    int u;
    int q;
    int f;
    int r;
    
    max_err[0] = net->gng_err[0];
    q = 0;
    min_u = net->gng_u[0];
    u = 0;
    net->movedis[0] = 0.0;
    
    
    
    for(j=0;j<DIM;j++){
        if(j < 3){
            net->dir[0][j] = net->node[0][j] - net->prenode[0][j];
            net->movedis[0] += net->dir[0][j]*net->dir[0][j];
        }
        net->prenode[0][j] = net->node[0][j];
    }
    
    for (i=1;i<net->node_n;i++){
        net->movedis[i] = 0.0;
        for(j=0;j<DIM;j++){
            if(j < 3){
                net->dir[i][j] = net->node[i][j] - net->prenode[i][j];
                net->movedis[i] += net->dir[i][j]*net->dir[i][j];
            }
            net->prenode[i][j] = net->node[i][j];
        }
        //        if(max_err[0] < net->gng_err[i]*15 && rangeArea(net->node[i][0],net->node[i][1],net->node[i][2],0,30,170,250,250,50)==1){
        //            max_err[0] = net->gng_err[i]*15;
        //            q = i;
        //        }else if(max_err[0] < net->gng_err[i]){
        //            max_err[0] = net->gng_err[i];
        //            q = i;
        //        }
        if(max_err[0] < net->gng_err[i] * pow((4*net->strength[i]),4)){
            max_err[0] = net->gng_err[i] * pow((4*net->strength[i]),4);
            q = i;
        }
        
        if(min_u > net->gng_u[i]){
            min_u = net->gng_u[i];
            u = i;
        }
    }
    max_err[1] = 0;
    f = GNGN;
    for (i=0;i<net->node_n;i++){
        if(net->edge[q][i] == 1 && q != i){
            //            if(net->gng_err[i]*15 > max_err[1] && rangeArea(net->node[i][0],net->node[i][1],net->node[i][2],0,30,170,250,250,50)==1){
            //                max_err[1] = net->gng_err[i]*15;
            //                f = i;
            //            }else if(net->gng_err[i] > max_err[1]){
            //                max_err[1] = net->gng_err[i];
            //                f = i;
            //            }
            if(net->gng_err[i] * pow((4*net->strength[i]),4) > max_err[1]){
                max_err[1] = net->gng_err[i] * pow((4*net->strength[i]),4);
                f = i;
            }
        }
    }
    r = net->node_n;
    for(i=0;i<DIM;i++){
        net->node[r][i] = 0.5*(net->node[q][i] + net->node[f][i]);
        if(i < 3){
            net->prenode[r][i] = net->node[r][i];
            net->dir[r][i] = 0.0;
        }
    }
    net->edge[q][f] = 0;
    net->edge[f][q] = 0;
    net->age[q][f] = 0;
    net->age[f][q] = 0;
    
    net->edge[q][r] = 1;
    net->edge[r][q] = 1;
    net->age[q][r] = 0;
    net->age[r][q] = 0;
    
    net->edge[r][f] = 1;
    net->edge[f][r] = 1;
    net->age[r][f] = 0;
    net->age[f][r] = 0;
    
    net->gng_err[q] -= 0.5*net->gng_err[q];
    net->gng_err[f] -= 0.5*net->gng_err[f];
    
    net->gng_u[q] -= 0.5*net->gng_u[q];
    net->gng_u[f] -= 0.5*net->gng_u[f];
    
    net->gng_err[r] = net->gng_err[q];
    net->gng_u[r] = net->gng_u[q];
    net->node_n = r+1;
    
    net->strength[r] = 1;
    gng_calc_strength(net, r);
    //    if(rangeArea(net->node[u][0],net->node[u][1],net->node[u][2],0,30,170,250,250,50)==1)
    //        net->K = 10000;
    //    else
    double K = net->K;// * pow((4*net->strength[u]),4);
    if(max_err[0] > K*min_u && flag == 1){
        node_delete(net, u);
        //        if(net->node[u][1] < -50){
        //            printf("test");
        //        }
        
    }
//    printf("node num = %d\n", net->node_n);
}
void node_add_gng(struct gng *net, int flag)
{
    int i,j;
    double max_err[2];
    double min_u;
    int u;
    int q;
    int f;
    int r;
    
    max_err[0] = net->gng_err[0];
    q = 0;
    min_u = net->gng_u[0];
    u = 0;
    net->movedis[0] = 0.0;
    
    
    
    for(j=0;j<DIM;j++){
        if(j < 3){
            net->dir[0][j] = net->node[0][j] - net->prenode[0][j];
            net->movedis[0] += net->dir[0][j]*net->dir[0][j];
        }
        net->prenode[0][j] = net->node[0][j];
    }
    
    for (i=1;i<net->node_n;i++){
        net->movedis[i] = 0.0;
        for(j=0;j<DIM;j++){
            if(j < 3){
                net->dir[i][j] = net->node[i][j] - net->prenode[i][j];
                net->movedis[i] += net->dir[i][j]*net->dir[i][j];
            }
            net->prenode[i][j] = net->node[i][j];
        }
//        if(max_err[0] < net->gng_err[i]*net->strength[i]){
//            max_err[0] = net->gng_err[i]*net->strength[i];
//            q = i;
//        }
        if(max_err[0] < net->gng_err[i]){
            max_err[0] = net->gng_err[i];
            q = i;
        }
        
        if(min_u > net->gng_u[i]){
            min_u = net->gng_u[i];
            u = i;
        }
    }
    max_err[1] = 0;
    f = GNGN;
    for (i=0;i<net->node_n;i++){
        if(net->edge[q][i] == 1 && q != i){
            if(net->gng_err[i] > max_err[1]){
                max_err[1] = net->gng_err[i];
                f = i;
            }
        }
    }
    r = net->node_n;
    for(i=0;i<DIM;i++){
        net->node[r][i] = 0.5*(net->node[q][i] + net->node[f][i]);
        if(i < 3){
            net->prenode[r][i] = net->node[r][i];
            net->dir[r][i] = 0.0;
        }
    }
    net->edge[q][f] = 0;
    net->edge[f][q] = 0;
    net->age[q][f] = 0;
    net->age[f][q] = 0;
    
    net->edge[q][r] = 1;
    net->edge[r][q] = 1;
    net->age[q][r] = 0;
    net->age[r][q] = 0;
    
    net->edge[r][f] = 1;
    net->edge[f][r] = 1;
    net->age[r][f] = 0;
    net->age[f][r] = 0;
    
    net->gng_err[q] -= 0.5*net->gng_err[q];
    net->gng_err[f] -= 0.5*net->gng_err[f];
    
    net->gng_u[q] -= 0.5*net->gng_u[q];
    net->gng_u[f] -= 0.5*net->gng_u[f];
    
    net->gng_err[r] = net->gng_err[q];
    net->gng_u[r] = net->gng_u[q];
    net->normAge[r] = 0;
    net->node_n = r+1;
    
    if(max_err[0] > net->K*min_u /*net->strength[u]/5*/ && flag == 1){
        node_delete(net, u);
    }
}

void node_delete_u(struct gng *net)
{
    int i;
    double max_err;
    double min_u;
    int u;
    int q;
    
    max_err = net->gng_err[0];
    q = 0;
    min_u = net->gng_u[0];
    u = 0;
    for (i=1;i<net->node_n;i++){
        if(max_err < net->gng_err[i]){
            max_err = net->gng_err[i];
            q = i;
        }
        
        if(min_u > net->gng_u[i]){
            min_u = net->gng_u[i];
            u = i;
        }
    }
    
    if(max_err > net->K*min_u){
        node_delete(net, u);
    }
    
    
}


int calc_age(int s1, int s2, struct gng *net)
{
    int i,j;
    int d_flag = 0;
    const int MAX_AGE[4] = {88, 40, 30, 15};
    if(net->edge[s1][s2] != 1){
        net->edge[s1][s2] = 1;
        net->edge[s2][s1] = 1;
    }
    
    net->age[s1][s2] = 0;
    net->age[s2][s1] = 0;
    
    for(i=0;i<net->node_n;i++){
        d_flag = 0;
        if(i != s1){
            if(net->edge[s1][i] == 1 && i != s2){
                net->age[s1][i]++;
                net->age[i][s1]++;
                if(MAX_AGE[net->layer] < net->age[s1][i]){
                    net->age[s1][i] = 0;
                    net->age[i][s1] = 0;
                    net->edge[s1][i] = 0;
                    net->edge[i][s1] = 0;
                    for(j=0;j<net->node_n;j++)
                        if(net->edge[j][i] == 1){
                            d_flag = 1;
                            break;
                        }
                    
                    if(d_flag == 0 && net->node_n > 2){
                        net->node_n = node_delete(net, i);
                        if(s1 > i)
                            s1--;
                        if(s2 > i)
                            s2--;
                        i--;
                    }
                }
            }
        }
    }
    
    return net->node_n;
}

void gng_learn_local_fcm(struct gng *net, int s1, int s2, double v[], double dis[])
{
    int i,j;
    double e1 = 0;
    int cct = 0;
    int *clist = (int *)malloc(sizeof(int)*net->node_n);
    double *mu = (double *)malloc(sizeof(double)*net->node_n);
    const double m = 2.0;
    const double ETA = 0.2;
    
    gng_calc_strength(net, s1);
    if(dis[s1] != 0){
        clist[cct] = s1;
        cct++;
    }else{
        e1 = ETA;
        
        for(i=0;i<DIM;i++){
            net->node[s1][i] += e1*(v[i]-net->node[s1][i]);
            if (net->node[s1][i] > 10000 || net->node[s1][i] < -10000) {
                printf("test");
            }
        }
        
        free(mu);
        free(clist);
        return;
    }
    
    if(dis[s2] != 0){
        clist[cct] = s2;
        cct++;
    }
    
    for(i=0;i<net->node_n;i++){
        if(net->edge[s1][i] == 1 && s1 != i && s2 != i){
            if(dis[i] != 0){
                clist[cct] = i;
                cct++;
            }
        }
    }
    
    for(i=0;i<cct;i++){
        mu[i] = 0.0;
        for(j=0;j<cct;j++){
            mu[i] += pow(dis[clist[i]]/dis[clist[j]], 1.0/(m-1.0));
        }
        mu[i] = 1.0/mu[i];
        e1 = ETA;
        for(j=0;j<m;j++)
            e1 *= mu[i];
        
        for(j=0;j<DIM;j++)
            net->node[clist[i]][j] += e1*(v[j]-net->node[clist[i]][j]);
        
        gng_calc_strength(net, clist[i]);
    }
    
    free(mu);
    free(clist);
}




double time_strength = 0;

double Cfoc[3];

// calculate strength of GNG

void gng_calc_strength(struct gng *net, int s){
    int i,j;
    double* v = (double *) malloc(sizeof (double ) * 3);
    int cN = 0;
    v[0]=0;v[1]=0;v[2]=0;
    double va;

    clock_t start, end;
    double cpu_time_used;

    start = clock();

    va = abs(norm(v, 3));
    if(va > 1) va = 1;

    va = 0;
//    double* P = (double *) malloc(sizeof (double ) * 3);
//    P[0] = net->node[s][0];
//    P[1] = net->node[s][1];
//    P[2] = net->node[s][2];
//
//    double nd = norm(vectorSubtractionD(P, Cfoc, 3),3);
//    if(nd < R_STRE * 3){
//        for(j=0; j<net->susNode_n; j++){
//            double n = norm(vectorSubtractionD(P, net->node[net->susNode[j]], 3),3);
//            if(n < R_STRE){
//                va = 5;
//                j = GNGN;
//            }
//        }
//    }
//
//    end = clock();
//    cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;
//    time_strength += cpu_time_used;
//    if(net->normTriangle[s][0] == 6) net->strength[s] = 10 + va;
//    else if(net->normTriangle[s][0] == 2) net->strength[s] = 10 + va;
//    else if(net->normTriangle[s][0] != 1) net->strength[s] = 1 + va;
//    net->strength[s] = 1;

    free(v);
//    free(P);

}

int last_node = 1;
int PCf[1000000];
int countCf = 0;

void learning_epoch(struct gng *net, double P[120000][DIM], int dmax, int ramda, double lrate, int flag, int dflag)
{
    int i,j,k;
    int s1 = 0,s2 = 0;
    double mindis = 0, mindis2 = 0, dis[GNGN];
    int t_countCF = 0;
    int l[ramda];
    double **v;
    v = malloc2d_double(dmax, DIM);
    for(i=0;i<ramda;i++){
        if(countCf == 0 || t_countCF > countCf){
            l[i] = (int)((double)dmax*rnd());
            if(l[i] >= dmax)
                l[i] = dmax - 1;
            if(l[i] < 0)
                l[i] = 0;
        }else{
            int a = (int)((double)countCf*rnd());
            if(a >= countCf)
                a = countCf - 1;
            if(a < 0)
                a = 0;
            l[i] = PCf[a];
            t_countCF++;
        }
    }
    
    for(k=0;k<ramda;k++){
        dis[0] = 0.0;
        
        v[l[k]][0] = P[l[k]][0];
        v[l[k]][1] = P[l[k]][1];
        v[l[k]][2] = P[l[k]][2];
        for(i=0;i<3;i++){
            dis[0] += net->weight[i]*(net->node[0][i] - v[l[k]][i])*(net->node[0][i] - v[l[k]][i]);
        }
        
        mindis = dis[0];
        s1 = 0;
        
        for (i=1;i<net->node_n;i++){
            dis[i] = 0;
            
            for(j=0;j<3;j++)
                dis[i] += net->weight[j]*(net->node[i][j] - v[l[k]][j])*(net->node[i][j] - v[l[k]][j]);
            if(dis[i] < mindis){
                mindis = dis[i];
                s1 = i;
            }
        }
        
        if(s1 != 0){
            mindis2 = dis[0];
            s2 = 0;
        }else{
            mindis2 = dis[1];
            s2 = 1;
        }
        
        for (i=0;i<net->node_n;i++){
            if(dis[i] < mindis2 && i != s1){
                mindis2 = dis[i];
                s2 = i;
            }
        }
        
        net->gng_err[s1] += mindis;
        net->gng_u[s1] += mindis2-mindis;
        
//        net->wct[s1]++;
//        net->wct[s2]++;
        
        gng_learn(net, s1, v[l[k]], lrate);
        net->node_n = calc_age(s1, s2, net);
        if(dflag == 1){
            
            discount_err_gng(net);
            if(net->node_n > 10 && flag == 1){
                node_delete_u(net);
            }
        }
    }
    
    free2d_double(v);
}
void gng_learn(struct gng *net, int s1, double v[], double e1)
{
    int i,j;
    double e2=e1/100.0;     //学習係数
    
    //第1勝者，第2勝者ノードの更新を行う
    for(j=0;j<DIM;j++){
        net->node[s1][j] += e1*(v[j] - net->node[s1][j]);
    }
    
    //第1勝者ノードと隣接関係を持つノードの更新を行う
    for(i=0;i<net->node_n;i++){
        if(net->edge[s1][i] == 1){
            for(j=0;j<DIM;j++)
                net->node[i][j] += e2*(v[j] - net->node[i][j]);
        }
    }
}

int gng_main(struct gng *net, double P[120000][DIM], int dmax, int tt, int flag, int add, double Cf[])
{
    int i,j;
    const int ramda[4] = {500, 200 , 200, 100};
    //int l[3000];
    
    const clock_t begin_time = clock();
    
//    for(i=0; i<net->node_n; i++){
//        net->strength[i] = 1;
//        double n = norm( vectorSubtractionD(net->node[i], Cf, 3),3);
//        if(n < 0.4){
//            net->strength[i] = 15;
//        }
//    }
//    add = 0;
    int maxData = (int)(0.9 * (float)(ramda[net->layer]))/((float)(net->susNode_n));
    int cData[net->susNode_n];
    for(j=0; j<net->susNode_n; j++){
        cData[j] = 0;
    }
    if(add == 2){
        countCf = 0;
        int st = 30;
        for(i=0; i<dmax; i+=(int)(st + st*rnd())){
//            double nd = norm( vectorSubtractionD(P[i], Cf, 3),3);
//            if(nd < R_STRE * 3){
            int det = 0;
                for(j=0; j<net->susNode_n; j++){
                    if(cData[j] < maxData){
                        double n = norm( vectorSubtractionD(P[i], net->node[net->susNode[j]], 3) ,3);
                        if(n < net->normTriangle[net->susNode[j]][1]){
                            PCf[countCf] = i;
                            countCf++;
    //                        if(st == 10)
    //                            i -= 10;
                            st = 2;
                            det = 1;
                            cData[j]++;
                            j = GNGN;
                        }
                    }
                }
            if(det == 0){
                st = 10;
            }
//            }
        }
    }else if(add == 0){
        countCf = 0;
    }
    printf("search %.6f\n",float( clock () - begin_time ) /  CLOCKS_PER_SEC);
    
    
    
//    learning_epoch(net, P, dmax, ramda[net->layer]/4, 0.085, flag, 0);  //Coarse tuning
    learning_epoch(net, P, dmax, ramda[net->layer]/4, 0.1, flag, 0);  //Coarse tuning
    learning_epoch(net, P, dmax, ramda[net->layer], 0.0075, flag, 1);
    
    last_node = net->node_n;
    if (last_node != net->node_n) {
        printf("test");
    }
    
    int a = 2+(GNGN - net->node_n)/100;
    for(int i=0; i<a; i++){
        if(net->node_n < GNGN-1)
            node_add_gng(net, flag);
//
        if(net->node_n < GNGN-1)
            node_add_gng2(net, flag);
    }
//    if(net->node_n < GNGN-1)
//        node_add_gng2(net, flag);
//
//    if(net->node_n < GNGN-1)
//        node_add_gng(net, flag);
//
//    if(net->node_n < GNGN-1)
//        node_add_gng(net, flag);
    
//    free(v);
    
    learning_epoch(net, P, dmax, 2*ramda[net->layer], 0.0001, flag, 0);   //Fine tuning
    
    return net->node_n;
}

void gng_learn_local(struct gng *net, int s1, int s2, double *v, double *h, double **h2, double *dis)
{
    int i,j;
    double e1;
    int cct = 0;
    int *clist = (int *)malloc(sizeof(int)*net->node_n);
    double *mu = (double *)malloc(sizeof(double)*net->node_n);
    const int m = 2;
    const double ETA = 1.0;
    
    if(dis[s1] != 0){
        clist[cct] = s1;
        cct++;
    }else{
        e1 = ETA;
        h[s1] += e1;
        
        for(i=0;i<DIM;i++){
            h2[s1][i] += e1*(v[i]-net->node[s1][i]);
        }
        
        free(mu);
        free(clist);
        return;
    }
    
    if(dis[s2] != 0){
        clist[cct] = s2;
        cct++;
    }
    
    for(i=0;i<net->node_n;i++){
        if(net->edge[s1][i] == 1 && s1 != i && s2 != i){
            if(dis[i] != 0){
                clist[cct] = i;
                cct++;
            }
        }
    }
    
    for(i=0;i<cct;i++){
        mu[i] = 0.0;
        for(j=0;j<cct;j++){
            mu[i] += pow(dis[clist[i]]/dis[clist[j]], 1.0/((double)m-1.0));
        }
        mu[i] = 1.0/mu[i];
        e1 = ETA;
        for(j=0;j<m;j++)
            e1 *= mu[i];
        h[clist[i]] += e1;
        for(j=0;j<DIM;j++)
            h2[clist[i]][j] += e1*(v[j]-net->node[clist[i]][j]);
    }
    
    free(mu);
    free(clist);
}

int calc_age_local(int s1, int s2, struct gng *net, int **edge, int **age)
{
    int i;
    
    edge[s1][s2]++;
    edge[s2][s1]++;
    for(i=0;i<net->node_n;i++){
        if(i != s1 && i != s2){
            if(net->edge[s1][i] == 1){
                age[s1][i]++;
                age[i][s1]++;
            }
        }
    }
    
    return net->node_n;
}
