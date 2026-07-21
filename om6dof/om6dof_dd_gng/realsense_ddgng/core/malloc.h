/*
 *  malloc.h
 *  Cluster
 *
 *  Created by Naoyuki Kubota on 14/07/01.
 *  Copyright 2014 首都大学東京. All rights reserved.
 *
 */

void free2d_double(double ** a);
double **malloc2d_double(int x, int y);

void free2d_int(int ** a);
int **malloc2d_int(int x, int y);

char **malloc2d_char(int x, int y);