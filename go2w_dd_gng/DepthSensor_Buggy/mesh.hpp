//
//  mesh.hpp
//  DepthSensor_Buggy
//
//  Created by Azhar Aulia Saputra on 2020/09/19.
//  Copyright © 2020 Azhar Aulia Saputra. All rights reserved.
//

#ifndef mesh_hpp
#define mesh_hpp

#include <stdio.h>
void triangle_search_with_range(struct gng *net, int t0, int t1);
void sphereOfTriangle(struct gng *net);
void reconstructEdge(struct gng *net, int t0, int t1);
void search_connection_num(struct gng *net);
#endif /* mesh_hpp */
