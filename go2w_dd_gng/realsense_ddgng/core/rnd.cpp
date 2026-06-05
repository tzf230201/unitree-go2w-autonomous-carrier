/*
 *  rnd.c
 *  Claster
 *
 *  Created by Naoyuki Kubota on 12/05/18.
 *  Copyright 2012 首都大学東京. All rights reserved.
 *
 */

#include "rnd.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

double rnd() /* uniform random number  */
{
	return((double)(rand()%30001)/30000.0);
}


double rndn()            /*   normal random number */
{
	return (rnd()+rnd()+rnd()+rnd()+rnd()+rnd()+
			rnd()+rnd()+rnd()+rnd()+rnd()+rnd()-6.0);
}

double calc_inner(double *a, double *b, int dim)
{
    int i;
    double result = 0.0;
    
    for(i=0;i<dim;i++)
        result += a[i]*b[i];
    
    return result;
}
