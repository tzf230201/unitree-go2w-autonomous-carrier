//
//  mesh.cpp
//  DepthSensor_Buggy
//
//  Created by Azhar Aulia Saputra on 2020/09/19.
//  Copyright © 2020 Azhar Aulia Saputra. All rights reserved.
//

#include "mesh.hpp"
#include "gng.hpp"
#include "main.h"
#include "projection.h"
int map_node_delete(struct gng *net, int i)
{
    int j,k,l;
    
    for(j=i;j<net->n_mapData[0];j++){
        for(l=0;l<9;l++){
            net->mapData[0][j][l] = net->mapData[0][j+1][l];
        }
        for(k=0;k<net->n_mapData[0];k++){
            if(k < i){
                net->mapEdge[0][j][k] = net->mapEdge[0][j+1][k];
                net->mapEdge[0][k][j] = net->mapEdge[0][k][j+1];
            }else{
                net->mapEdge[0][j][k] = net->mapEdge[0][j+1][k+1];
            }
        }
    }
    
    for(k=0;k<net->n_mapData[0];k++){
        net->mapEdge[0][net->n_mapData[0]][k] = 0;
        net->mapEdge[0][k][net->n_mapData[0]] = 0;
    }
    net->n_mapData[0]--;
    if (net->n_mapData[0] <= 0) {
        printf("test");
    }
    return net->n_mapData[0];
}
void triangle_search_with_range(struct gng *net, int t0, int t1)
{
    int i,j,k;
    int ct = 0;
    int count = 0;
    for(i=t0;i<t1;i++){
        int tc_i = 0;
        for(j=0;j<net->n_mapData[0];j++){
            for(k=j+1;k<net->n_mapData[0];k++){
                if(net->mapData[0][i][0] == 0 && net->mapData[0][i][1] == 0 && net->mapData[0][i][2] == 0) continue;
                if(net->mapData[0][j][0] == 0 && net->mapData[0][j][1] == 0 && net->mapData[0][j][2] == 0) continue;
                if(net->mapData[0][k][0] == 0 && net->mapData[0][k][1] == 0 && net->mapData[0][k][2] == 0) continue;
//                if(i > GNGN || j > GNGN || k > GNGN){
//                    printf("%d,%d,%d\n",i,j,k);
//                    continue;
//                }
                if((net->mapEdge[0][i][j] == 1 || net->mapEdge[0][j][i] == 1) && (net->mapEdge[0][i][k] == 1 || net->mapEdge[0][k][i] == 1) && (net->mapEdge[0][k][j] == 1 || net->mapEdge[0][j][k] == 1) && i!=j && j !=k && k != i){
                    net->triangle[ct][0] = i;
                    net->triangle[ct][1] = j;
                    net->triangle[ct][2] = k;
                    ct++;
                    if(net->mapData[0][j][6] != 6 && net->mapData[0][k][6] != 6)
                        tc_i++;
                }
            }
        }
        if(tc_i == 0){
//            count++;
//            t1 = map_node_delete(net,i);
//            i--;
        }
    }
    net->triangle_n = ct;
}
void search_connection_num(struct gng *net)
{
    int i,j,k;
    int ct = 0;
    int count = 0;
    for(i=0;i<net->n_mapData[0];i++){
        int tc_i = 0;
        for(j=0;j<net->n_mapData[0];j++){
            if((net->mapEdge[0][i][j] == 1 || net->mapEdge[0][j][i] == 1) && i!=j){
                if(net->mapData[0][j][6] != 6 && net->mapData[0][k][6] != 6)
                    tc_i++;
            }
        }
        if(tc_i == 0){
            count++;
            map_node_delete(net,i);
            i--;
        }
    }
}
void deleteTriangle(struct gng *net, int i){
    int j,k,l;
    for(j=i;j<net->triangle_n;j++){
        for(l=0;l<3;l++){
            net->triangle[j][l] = net->triangle[j+1][l];
        }
    }
    net->triangle_n --;
}
void sphereOfTriangle(struct gng *net){
    int nTri = 0;
    for(int i = 0; i < net->triangle_n; i++){
        int p1 = net->triangle[i][0];
        int p2 = net->triangle[i][1];
        int p3 = net->triangle[i][2];
        double *v1 = vectorSubtraction(net->mapData[0][p1], net->mapData[0][p2], 3);
        double *r1 = vectorAdd(net->mapData[0][p2], vectorScale(0.5, v1, 3), 3);
        double *v2 = vectorSubtraction(net->mapData[0][p1], net->mapData[0][p3], 3);
        double *r2 = vectorAdd(net->mapData[0][p3], vectorScale(0.5, v2, 3), 3);
        double *n = crossProductD(v1, v2);
        double *nv1 = crossProductD(n, v1);
        double *nv2 = crossProductD(n, v2);
           
        double a;
        double *xv1v2 = crossProductD(nv1, nv2);
        double *p1p2 = vectorSubtraction(r2, r1, 3);
        double *xp1p2V2 = crossProductD(p1p2, nv2);
        if(xv1v2[0] != 0){
            a = xp1p2V2[0]/xv1v2[0];
        }else if(xv1v2[1] != 0){
            a = xp1p2V2[1]/xv1v2[1];
        }else if(xv1v2[2] != 0){
            a = xp1p2V2[2]/xv1v2[2];
        }
           
        double *C = vectorAdd(r1, vectorScale(a, nv1, 3), 3);
        double r = norm(vectorSubtraction(C, net->mapData[0][p1], 3), 3);
           
        int cInPoint = 0;
        for(int j=0; j<net->n_mapData[0]; j++){
            if(j != p1 && j != p2 && j != p3){
                double *a = vectorSubtraction(C, net->mapData[0][j], 3);
                if(r > norm(a, 3)){
                    cInPoint++;
                }
                free(a);
            }
        }
        if(cInPoint == 0){
            nTri++;
        }else{
            deleteTriangle(net, i);
            i--;
        }
//        if(Mesh->triangle[i][3] >= 1)
//            drawPoint(C, 20);
        
        
           
          // printf("%f\t%f\t%f\n", xp1p2V2[0]/xv1v2[0], xp1p2V2[1]/xv1v2[1], xp1p2V2[2]/xv1v2[2]);
           
        free(r1);
        free(r2);
        free(v1);
        free(v2);
        free(n);
        free(nv1);
        free(nv2);
        free(xv1v2);
        free(p1p2);
        free(xp1p2V2);
        free(C);
    }
    
}
void reconstructEdge(struct gng *net, int t0, int t1){
    int i,j,a,b,k;
    for(i=t0;i<t1;i++){
        for(j=0;j<net->n_mapData[0];j++){
            net->mapEdge[0][i][j] = 0;
            net->mapEdge[0][j][i] = 0;
        }
    }
    for(int i = 0; i < net->triangle_n; i++){
        for(k=0;k<3;k++){
            a = net->triangle[i][k%3];
            b = net->triangle[i][(k+1)%3];
            net->mapEdge[0][a][b] = 1;
            net->mapEdge[0][b][a] = 1;
        }
    }
}
