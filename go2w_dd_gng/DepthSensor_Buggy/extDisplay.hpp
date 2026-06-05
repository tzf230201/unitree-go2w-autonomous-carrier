//
//  extDisplay.hpp
//  DepthSensor_Buggy
//
//  Created by Azhar Aulia Saputra on 2020/08/18.
//  Copyright © 2020 Azhar Aulia Saputra. All rights reserved.
//

#ifndef extDisplay_hpp
#define extDisplay_hpp

#include <stdio.h>
void MotionEventHandler(int x, int y);
void MouseEventHandler(int button, int state, int x, int y);
void keyboard(unsigned char key, int x, int y);
void reshape(int width, int height);
void display();
void initMotionModel();
void updateGLView(float hrp[], float xyz[]);
void initialExtGL();
void drawLine(double p0[], double p1[], double s);
void drawPoint(double p[], double s);



#endif /* extDisplay_hpp */
