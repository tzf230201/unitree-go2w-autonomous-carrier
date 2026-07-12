/* Minimal common.h shim for vendored drawstuff build.
 * The original ODE private common.h drags in the whole ODE internal toolkit;
 * drawstuff/src/windows.cpp only references dEpsilon from it.
 */
#ifndef DS_COMMON_H_SHIM
#define DS_COMMON_H_SHIM

#include <float.h>

#ifdef dSINGLE
#define dEpsilon FLT_EPSILON
#else
#define dEpsilon DBL_EPSILON
#endif

#endif
