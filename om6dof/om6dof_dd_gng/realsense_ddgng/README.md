# OM6DOF DD-GNG RealSense

Run the **DD-GNG** algorithm from the `DepthSensor_Buggy` simulation on the live
**Intel RealSense D435i** and overlay the resulting growing-neural-gas network on
the camera's 2D RGB view.

The GNG core is the *unmodified* simulation code (`gng.cpp`, Fritzke 1995
parameters). Only the input and output are replaced:

| | Simulation | Here |
|---|---|---|
| Input | ODE ray-scan point cloud | RealSense depth → deprojected 3D points |
| Output | OpenGL second window | OpenCV overlay on the RGB frame |

## Architecture

```
ddgng_realsense.py  (pyrealsense2 + OpenCV)
        │  ctypes
        ▼
libddgng.so  ←  core/{gng,projection,rnd,malloc}.cpp + ddgng_wrapper.cpp
```

- `core/` holds **copies** of the GNG core from `../DepthSensor_Buggy/`, with
  `#include "main.h"` redirected to `core/ddgng_compat.h` (constants only, no ODE/GL)
  and the `extDisplay.hpp` / `surfMatching.hpp` includes stubbed.
- `core/ddgng_wrapper.cpp` exposes a tiny C API (`ddgng_create/feed/get_nodes/
  get_edges/...`) and defines the few simulator symbols the core still references
  (`SLAMpos/SLAMrot`, `EulerToMatrix`, no-op `drawLine`).
- The original simulation in `../DepthSensor_Buggy/` is **untouched**.

## Build

```bash
cmake -S . -B build_om6dof
cmake --build build_om6dof -j
```
Produces `build_om6dof/libddgng.so`. Needs OpenGL/GLUT dev (already installed) only so
the unused GL symbols in the GNG helpers resolve at link time — no GL window is
created at runtime.

## Run

Needs the venv with `pyrealsense2` + `cv2` (`~/ggcnn_env`) and a display:

```bash
DISPLAY=:1002 LD_LIBRARY_PATH=build_om6dof ~/ggcnn_env/bin/python ddgng_realsense.py
```

Point the camera at a scene: red nodes + green edges grow over the surfaces in
the RGB view. HUD shows `nodes/edges/pts/fps`. Press `ESC` or `q` to quit.

## Tuning (pre-processing only — never touch the GNG parameters)

Edit the constants at the top of `ddgng_realsense.py`:

- `PIXEL_STEP` — sub-sample stride (bigger = fewer points = faster, coarser).
- `Z_MIN`, `Z_MAX` — depth clamp in metres.
- `GNG_ITERS` — learning passes per frame (sim uses 4).

If the GNG looks wrong on real data, fix it here (sampling / range / centroid),
**not** in `core/gng.cpp` — the Fritzke 1995 parameters are intentionally fixed.

## Notes

- Overlay uses the colour intrinsics for both deprojection and re-projection
  (plain pinhole), so it stays self-consistent; depth is aligned to colour first.
- `~12 fps` at 640×480 with `PIXEL_STEP=6` on the Orin NX (GNG capped at 500 nodes).
