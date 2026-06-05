/*
 *  malloc.c
 *  Cluster
 *
 *  Created by Naoyuki Kubota on 14/07/01.
 *  Copyright 2014 首都大学東京. All rights reserved.
 *
 */

#include "malloc.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void free2d_double(double ** a)	//2次元配列の開放
{
	free(a[0]);
	free(a);
}

double **malloc2d_double(int x, int y)	//2次元配列の初期化
{
	double **a;
	int i;
	a = (double **)malloc(sizeof(double *)*(x+1));
	a[0] = (double *)malloc(sizeof(double)*(y+1)*(x+1));
	for(i=1;i<(x+1);i++) a[i] = a[0] + i*(y+1);
	memset(a[0], 0,sizeof(a[0]));
	return a;
}

void free2d_int(int ** a)
{
	free(a[0]);
	free(a);
}

int **malloc2d_int(int x, int y)
{
	int **a;
	int i;
	a = (int **)malloc(sizeof(int *)*(x+1));
	a[0] = (int *)malloc(sizeof(int)*(y+1)*(x+1));
	for(i=1;i<(x+1);i++) a[i] = a[0] + i*(y+1);
	memset(a[0], 0,sizeof(a[0]));
	return a;
}

void free2d_char(char ** a)
{
	free(a[0]);
	free(a);
}

char **malloc2d_char(int x, int y)
{
	char **a;
	int i;
	a = (char **)malloc(sizeof(char *)*(x+1));
	a[0] = (char *)malloc(sizeof(char)*(y+1)*(x+1));
	for(i=1;i<(x+1);i++) a[i] = a[0] + i*(y+1);
	memset(a[0], 0,sizeof(a[0]));
	return a;
}