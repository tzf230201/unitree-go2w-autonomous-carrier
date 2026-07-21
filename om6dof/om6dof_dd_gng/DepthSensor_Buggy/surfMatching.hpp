//
//  surfMatching.hpp
//  DepthSensor_Buggy
//
//  Created by Azhar Aulia Saputra on 2020/08/25.
//  Copyright © 2020 Azhar Aulia Saputra. All rights reserved.
//

#ifndef surfMatching_hpp
#define surfMatching_hpp

#include <stdio.h>
void surfaceMatching(struct gng *net);
void printDataMap(struct gng *net);

double vectorToEulerTest();
void vectorToEuler(double v1[3], double v2[3], double out[3]);
void EulerToMatrix(
            double roll,     //Yaw   angle (radians)
            double pitch,   //Pitch angle (radians)
            double yaw,
                   double A[3][3] );  //Roll  angle (radians)
extern int stepMatch;
extern double SLAMpos[3];
extern double SLAMrot[3];
extern double dSLAMrot[3];
#endif /* surfMatching_hpp */
