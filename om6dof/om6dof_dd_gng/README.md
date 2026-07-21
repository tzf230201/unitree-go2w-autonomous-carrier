# om6dof_dd_gng

DD-GNG experiments and RealSense processing for the OM6DOF stack.

The project now lives under `om6dof/om6dof_dd_gng`; it is a standalone CMake
project rather than a ROS 2 package. It contains:

- `DepthSensor_Buggy/`: the original ODE/OpenGL simulation.
- `realsense_ddgng/`: the RealSense input and OpenCV overlay using the shared
  DD-GNG core.

Build the RealSense core from a clean build directory:

```bash
cmake -S realsense_ddgng -B realsense_ddgng/build_om6dof
cmake --build realsense_ddgng/build_om6dof -j
```

See `realsense_ddgng/README.md` for runtime dependencies and usage.

## Web monitor

The user service runs DD-GNG headlessly and publishes its annotated stream on
`/application_web_monitor/ddgng/image/compressed`:

```bash
install -m 0644 systemd/om6dof-dd-gng.service \
  ~/.config/systemd/user/om6dof-dd-gng.service
systemctl --user daemon-reload
```

Use **Start DD-GNG YOLO** / **Stop DD-GNG YOLO** in the Kublab web monitor.
The web service runs `dd_gng_yolo.py`; its semantic node metadata is published
on `/application_web_monitor/ddgng/labels`. DD-GNG and
OM6DOF perception both own the RealSense directly, so the systemd service
declares them as conflicting workloads; starting either one stops the other.

## DD-GNG + YOLO semantic nodes

`realsense_ddgng/dd_gng_yolo.py` combines the DD-GNG graph with YOLOX. A GNG
node receives a COCO label when its projected pixel intersects a YOLO box and
its 3D depth agrees with that object's foreground surface. This extra depth
gate prevents background nodes inside a 2D box from being labelled as the
object.

```bash
systemctl --user stop om6dof-dd-gng.service om6dof-perception.service
python3 realsense_ddgng/dd_gng_yolo.py \
  --headless \
  --ros-topic /om6dof_dd_gng_yolo/image/compressed \
  --labels-topic /om6dof_dd_gng_yolo/labels
```

The labels topic is JSON containing each matched node's index, YOLO class and
confidence, camera-frame XYZ coordinate, and projected UV pixel. Use
`--classes bottle,cup` to restrict labelling or `--hide-node-labels` to retain
semantic colours without drawing text beside every matched node.
