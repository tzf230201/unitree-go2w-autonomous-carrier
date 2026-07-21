//
//  ddgng_wrapper.cpp
//
//  Thin C API around the DD-GNG core (gng.cpp) so it can be driven from Python
//  via ctypes. Also provides the few simulator symbols the core references but
//  that we don't otherwise compile (SLAM globals, EulerToMatrix, drawLine).
//
#include "gng.hpp"          // struct gng, init_gng, gng_main, calc_node_normal_vector
#include <string.h>
#include <math.h>

// --- simulator symbols referenced by gng.cpp's unused code paths ------------
double SLAMpos[3] = {0, 0, 0};
double SLAMrot[3] = {0, 0, 0};

// Real (correct) rotation-matrix builder; only reached via the unused
// getGNGdata() SLAM path, kept correct for completeness.
void EulerToMatrix(double roll, double pitch, double yaw, double A[3][3]) {
    const double cr = cos(roll),  sr = sin(roll);
    const double cp = cos(pitch), sp = sin(pitch);
    const double cy = cos(yaw),   sy = sin(yaw);
    A[0][0] = cy*cp;  A[0][1] = cy*sp*sr - sy*cr;  A[0][2] = cy*sp*cr + sy*sr;
    A[1][0] = sy*cp;  A[1][1] = sy*sp*sr + cy*cr;  A[1][2] = sy*sp*cr - cy*sr;
    A[2][0] = -sp;    A[2][1] = cp*sr;             A[2][2] = cp*cr;
}

// No-op: only invoked from calc_node_normal_vector when showArrow != 0, which
// we never request. Drawing is done in Python (OpenCV), not GL.
void drawLine(double /*p0*/[], double /*p1*/[], double /*s*/) {}

// ---------------------------------------------------------------------------
//  C API
// ---------------------------------------------------------------------------
extern "C" {

// Create a fresh GNG network. Returns an opaque handle (struct gng*).
void *ddgng_create() {
    return (void *)init_gng();
}

// Feed one batch of 3D points and run `iters` GNG learning passes.
//   pts : contiguous array of n*3 doubles (x,y,z,...) in metres, sensor frame
//   cx,cy,cz : centre-of-focus passed to the core (Cfoc)
// Returns the current node count.
int ddgng_feed(void *handle, const double *pts, int n,
               double cx, double cy, double cz, int iters) {
    struct gng *net = (struct gng *)handle;
    if (!net || !pts || n <= 0) return net ? net->node_n : 0;

    static int tt = 1;            // monotonic time/iteration counter (tLoop)
    Cfoc[0] = cx; Cfoc[1] = cy; Cfoc[2] = cz;

    // gng_main treats P as double[][3]; our flat buffer is layout-compatible.
    double (*P)[DIM] = (double (*)[DIM])pts;
    for (int i = 0; i < iters; i++) {
        gng_main(net, P, n, tt, 1, 2, Cfoc);   // (net, P, dmax, tt, flag, add, Cf)
        tt++;
    }
    return net->node_n;
}

int ddgng_num_nodes(void *handle) {
    struct gng *net = (struct gng *)handle;
    return net ? net->node_n : 0;
}

// Copy node positions into out_xyz (size >= maxn*3). Returns nodes written.
int ddgng_get_nodes(void *handle, double *out_xyz, int maxn) {
    struct gng *net = (struct gng *)handle;
    if (!net || !out_xyz) return 0;
    int n = net->node_n;
    if (n > maxn) n = maxn;
    for (int i = 0; i < n; i++) {
        out_xyz[i*3 + 0] = net->node[i][0];
        out_xyz[i*3 + 1] = net->node[i][1];
        out_xyz[i*3 + 2] = net->node[i][2];
    }
    return n;
}

// Copy edges as (i,j) index pairs with i<j into out_pairs (size >= maxp*2).
// Returns number of pairs written.
int ddgng_get_edges(void *handle, int *out_pairs, int maxp) {
    struct gng *net = (struct gng *)handle;
    if (!net || !out_pairs) return 0;
    int cnt = 0;
    int n = net->node_n;
    for (int i = 0; i < n && cnt < maxp; i++) {
        for (int j = i + 1; j < n && cnt < maxp; j++) {
            if (net->edge[i][j]) {
                out_pairs[cnt*2 + 0] = i;
                out_pairs[cnt*2 + 1] = j;
                cnt++;
            }
        }
    }
    return cnt;
}

void ddgng_destroy(void *handle) {
    // init_gng allocates with the project's custom allocator; the simulation
    // itself never frees the net (single long-lived instance), so we mirror
    // that and simply drop the handle. Process exit reclaims the memory.
    (void)handle;
}

} // extern "C"
