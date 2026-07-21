# om6dof_perception

RealSense RGB-D perception for the OM6DOF arm. Local YOLOX-S detects a COCO
target class, OpenCV CSRT tracks it between detections, and aligned depth
produces its 3D position in `camera_color_optical_frame`. Ollama and a VLM are
not used by this node.

## Run

Download the Apache-2.0 OpenCV Zoo YOLOX-S ONNX model once on the NX:

```bash
mkdir -p ~/.cache/om6dof_perception
curl -L --fail \
  -o ~/.cache/om6dof_perception/yolox_s.onnx \
  https://github.com/opencv/opencv_zoo/raw/main/models/object_detection_yolox/object_detection_yolox_2022nov.onnx
echo "c5c2d13e59ae883e6af3b45daea64af4833a4951c92d116ec270d9ddbe998063  $HOME/.cache/om6dof_perception/yolox_s.onnx" \
  | sha256sum -c -
ros2 launch om6dof_perception perception.launch.py
```

Open the application web monitor at `http://<robot-ip>:8080`. The processed
RealSense perception card appears automatically because perception publishes
its JPEG overlay to
`/application_web_monitor/perception/image/compressed`. It disappears shortly
after perception stops. The Go2W built-in camera remains available separately
on `/application_web_monitor/image/compressed`.

## Control from the Kublab GUI

Install the perception user service on the NX. This does not require sudo and
works with the existing passwordless SSH connection from the Kublab AGX:

```bash
mkdir -p ~/.config/systemd/user
install -m 0644 \
  ~/ros2_ws/install/om6dof_perception/share/om6dof_perception/systemd/om6dof-perception-user.service \
  ~/.config/systemd/user/om6dof-perception.service
systemctl --user daemon-reload
```

The **OM6DOF perception** card in the Kublab dashboard can now start and stop
the user service over the configured `robot_ssh_host`. It also publishes a new
target description on `/om6dof_perception/set_target`. Do not enable this unit
at boot when another application normally owns the RealSense camera. User
lingering must be enabled (`loginctl show-user unitree -p Linger`); the current
NX configuration already reports `Linger=yes`.

Select another target at launch:

```bash
ros2 launch om6dof_perception perception.launch.py \
  target_description:="red cup on the table" target_class:=cup
```

Or change it while the node is running:

```bash
ros2 topic pub --once /om6dof_perception/set_target std_msgs/msg/String \
  "{data: 'red cup on the table'}"
```

Main outputs:

- `/om6dof_perception/target_point`
- `/om6dof_perception/target_bbox3d` (`[center_x, center_y, center_z, size_x, size_y, size_z]`)
- `/om6dof_perception/ee_point`
- `/om6dof_perception/relative` (`[dx, dy, dz, distance]`)
- `/om6dof_perception/distance` (`std_msgs/Float64`, meter)
- `/om6dof_perception/status`
- `/om6dof_perception/debug_image/compressed`
- `/application_web_monitor/perception/image/compressed`

`target_point` is the estimated object-volume centre, not merely the first
visible depth surface. The node isolates the foreground depth cluster inside
the YOLO box, measures its visible width/height, and infers the unseen depth
from the smaller visible extent. The inferred 3D dimensions are included in
the status JSON and drawn on the web overlay. Tune `bbox3d_depth_ratio` for an
object family whose depth differs substantially from its visible width.

The bundled model uses the 80 COCO classes. Free-form GUI text is reduced to a
known class (`red cup` -> `cup`, `glass jar` -> `bottle`). Because a robot
gripper is not a COCO class, EoE is estimated from two calibrated jaw pixels in
the fixed camera view: left `(200, 430)` and right `(540, 400)`. Their aligned
depth points are averaged in 3D. Override them with `ee_left_pixel` and
`ee_right_pixel` after the camera mount or image crop changes.

The node opens the RealSense directly. Stop other camera-owning applications
before starting it.

## Pickup from perception

The perception-driven pickup backend consumes
`/om6dof_perception/target_point`, rejects noisy depth clusters, transforms the
stable optical point through the calibrated wrist-camera extrinsic, and checks
both standoff and grasp poses with IK before allowing motion.

Install and start its user service on the NX:

```bash
install -m 0644 \
  ~/ros2_ws/install/om6dof_pick_and_place/share/om6dof_pick_and_place/systemd/om6dof-perception-pick-user.service \
  ~/.config/systemd/user/om6dof-perception-pick.service
systemctl --user daemon-reload
systemctl --user start om6dof-perception-pick.service
```

Run only after confirming F3 remote mode is OFF and the arm workspace is clear:

```bash
ros2 service call /direct_reachable std_srvs/srv/Trigger '{}'
ros2 service call /run_perception_pick std_srvs/srv/Trigger '{}'
```

Sequence: snapshot target -> reachability preflight -> ready -> gripper open ->
standoff -> advance -> gripper close -> retreat/lift -> place -> release ->
ready. The Kublab dashboard exposes the same guarded action as **Pickup
object**. An unreachable target is rejected before any trajectory is sent.
