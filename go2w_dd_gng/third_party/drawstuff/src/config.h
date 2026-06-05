/* Minimal config.h shim for vendored drawstuff build.
 * The original ODE config.h is autoconf-generated; drawstuff only uses
 * HAVE_APPLE_OPENGL_FRAMEWORK from it (to pick GL header style).
 * Leaving it undefined falls through to <GL/gl.h>, which is correct on
 * Windows/Linux. On macOS the project picks the Apple GLUT headers in its
 * own #ifdef __APPLE__ blocks, so this still works there.
 */
#ifndef DS_CONFIG_H_SHIM
#define DS_CONFIG_H_SHIM
#endif
