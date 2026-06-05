//
//  ddgng_compat.h
//
//  Drop-in replacement for the simulation's main.h when compiling ONLY the
//  DD-GNG core (gng.cpp + projection.cpp) into a standalone shared library.
//
//  The original main.h drags in ODE (ode/ode.h), drawstuff and the GLUT window
//  API plus dozens of simulation globals. The GNG algorithm itself uses none of
//  ODE — it only needs a handful of numeric constants and a few extern symbols.
//  This header provides exactly those, so the core builds without the simulator.
//
#ifndef ddgng_compat_h
#define ddgng_compat_h

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// A couple of projection.* helpers are typed with ODE's dReal. Under dDOUBLE
// (which we compile with) dReal is just double, so define it without ODE.
typedef double dReal;

// --- numeric constants copied verbatim from main.h (non-ODE) ---------------
#define RAD 180.0/3.14159265358979323846264338327950288
#define deg_to_rad    (float)0.0174532925199432957692369076848
#define rad_to_deg    (float)57.295779513082320876798154814105
#define rad 57.295779513082320876798154814105
#define phi 3.1415926535897932384626433832795
#define sp2 1.4142135623730950488016887242097

#define SNUM 3
#define GAIN 300
#define yNUM 1
#define xNUM 150

#define yMin -M_PI/4
#define yMax M_PI/4
#define xMin -0.41421
#define xMax 0.41421
#define xRes (xMax - xMin)/(xNUM-1)

// --- extern symbols the core references but that live in the simulator ------
// These are defined in ddgng_wrapper.cpp. They belong to code paths we do not
// drive (the SLAM capture in getGNGdata, and the optional arrow drawing in
// calc_node_normal_vector with showArrow=0), but the translation units must
// still compile and link.
#ifdef __cplusplus
extern "C" {
#endif
extern double SLAMpos[3];
extern double SLAMrot[3];
#ifdef __cplusplus
}
#endif

// EulerToMatrix: real implementation provided in the wrapper (used only by the
// unused getGNGdata path, but kept correct anyway).
void EulerToMatrix(double roll, double pitch, double yaw, double A[3][3]);
// drawLine: no-op stub in the wrapper (only reached when showArrow != 0).
void drawLine(double p0[], double p1[], double s);

#endif /* ddgng_compat_h */
