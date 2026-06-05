/*
 *  gng.h
 *  Claster
 *
 *  Created by Naoyuki Kubota on 12/05/18.
 *  Copyright 2012 首都大学東京. All rights reserved.
 *
 */


#define GNGN 500
#define CN 500
#define DIM 3

#define GNGNB 50
#define segmentNum 10
#define disNum 5
#define dirNum 16
#define R_STRE 0.1

#include "ddgng_compat.h"

struct gng {
    double node[GNGN][DIM];     //ノード位置
    
    int edge[GNGN][GNGN];       //エッジ(1:あり，0なし)
    int edgeTriangle[GNGN][GNGN];       //エッジ(1:あり，0なし)
    double normTriangle[GNGN][DIM];       //エッジ(1:あり，0なし)
    int normAge[GNGN];
    int age[GNGN][GNGN];        //エッジの年齢
    int node_n;                 //ノード数
    int susNode_n;                 //ノード数
    int susNode[GNGN];                 //ノード数
    double susLabelSize[GNGN][3][3];                 //ノード数
    int n_susLabel;                 //ノード数
    int susNodeLabel[GNGN];                 //ノード数
    
    int triangle[10*GNGN][5];
    double normVect[GNGN][5];
    double mapData[2][30 * GNGN][9];
    int mapEdge[2][30 * GNGN][30 * GNGN];
    int n_mapData[2];
//    int doubleCon[GNGN][5];
//    int square[GNGN][4];
    int intentional_n;
    int intNode_n;
    int triangle_n;
    int doubleCon_n;
    int square_n;
    int cluster[CN][GNGN];
    int cluster_num[CN];
    int cluster_ct;
    double weight[DIM];         //学習の重み
    double gng_err[GNGN];       //ノードの積算誤差
    double strength[GNGN];       //ノードの積算誤差
    double gng_u[GNGN];         //utility valiables
    struct gng *child;          //子ノード
    struct gng *parent;         //親ノード
    int layer;                  //層の数
    
    double delta[GNGN][DIM];    //更新量記憶用
    int edge_dimension[GNGN];    //エッジの次数
    
    //追加（2016/2/16）
    double prenode[GNGN][DIM];  //t-1のノード位置
    double dir[GNGN][3];        //移動方向
    double movedis[GNGN];       //移動距離の2乗
    int maxMDnode;           //最大の移動距離を持つノード番号
    int maxCVnode;           //最大の曲率を持つノード番号
    
    double cog[3];          //注視領域
    double sigma;           //注視領域の分散値
    
    double K;               //Utilityの係数
    
    double safedeg[GNGN];       //移動距離の2乗
    int label[CN];
    double label_node[CN][DIM];
    int label_list[CN][GNGN];
    int label_ct[CN];
    int label_edge[CN][CN];
    int label_num;
    
    int wct[GNGN];
    
    
    double maxE;
    int maxE_n;
    
    //追加
    double udrate;
    double novec[GNGN][3];
    double curv[GNGN];
};
extern double Cfoc[3];
extern int segmentInt[segmentNum];
extern struct gng *ocnet;
extern struct gng *intnet;
struct gng *init_gng();
int gng_main(struct gng *net, double v[][DIM],int tt, int dmax);
void gng_learn(struct gng *net, int s1, double v[], double e1);

void getGNGdata(struct gng *net, int c);
void gng_classification(struct gng *net);
void gng_triangulation(struct gng *net);
struct gng *hgng_main(struct gng *net, double v[][DIM],int tt, int dmax);
void connect_colordata(struct gng *net, int color[][3], int color_cluster[][3], int dmax);
void connect_distancedata(struct gng *net, double v[][DIM], int distance_cluster[], int dmax, int dim_s, int dim_f);

int gng_main(struct gng *net, double P[90000][DIM], int dmax, int tt, int flag, int add, double Cf[]);
void gng_triangle_search(struct gng *net);
void intentionalRawData(struct gng *net, double P[90000][DIM]);
void search_doubleConnection(struct gng *net);
void calc_node_normal_vector(struct gng *net, double *sP, int showArrow);
void gng_calc_strength(struct gng *net, int s);

//追加
int search_maxMD(struct gng *net);
int search_maxCurvature(struct gng *net);
int gng_main_c(struct gng *net, double v[][DIM],int tt, int dmax);
int search_maxSafedeg(struct gng *net);
void calc_safe_area(struct gng *net);
void calc_surface_label(struct gng *net);
int label_to_data(struct gng *net, double v[]);
void label_sort(struct gng *net);


extern double allDataCam[SNUM*yNUM*xNUM][DIM];

//int gng_main_oct_test(struct gng *net, Pointcloud &ptc, Pointcloud &cptc,int tt, int flag);
//
//int gng_batch_main_s(struct gng *net, Pointcloud &ptc, Pointcloud &cptc, int tt);
