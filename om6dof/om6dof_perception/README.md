# om6dof_perception

RealSense RGB-D perception for the OM6DOF arm. A VLM locates the requested
target and gripper, OpenCV CSRT tracks both objects, and aligned depth produces
their 3D positions in `camera_color_optical_frame`.

## Run

Make sure the Ollama service on the AGX is running and reachable from the NX:

```bash
curl http://192.168.123.99:11434/api/tags
ros2 launch om6dof_perception perception.launch.py
```

Open the application web monitor at `http://<robot-ip>:8080`. The processed
camera card appears automatically because perception publishes its JPEG overlay
to `/application_web_monitor/image/compressed`. It disappears shortly after
perception stops.

Select another target at launch:

```bash
ros2 launch om6dof_perception perception.launch.py \
  target_description:="red cup on the table"
```

Or change it while the node is running:

```bash
ros2 topic pub --once /om6dof_perception/set_target std_msgs/msg/String \
  "{data: 'red cup on the table'}"
```

Main outputs:

- `/om6dof_perception/target_point`
- `/om6dof_perception/ee_point`
- `/om6dof_perception/relative` (`[dx, dy, dz, distance]`)
- `/om6dof_perception/status`
- `/om6dof_perception/debug_image/compressed`
- `/application_web_monitor/image/compressed`

The node opens the RealSense directly. Stop other camera-owning applications
before starting it.

## Run the VLM on a remote Jetson AGX Orin

Keep this perception node on the Jetson Orin NX so RealSense, depth processing,
tracking, and ROS topics remain local. Only the occasional JPEG grounding
request is sent to Ollama on the AGX.

On the AGX, make Ollama listen on the robot LAN and ensure the model exists:

```bash
ollama pull qwen3-vl:8b-instruct-q4_K_M
sudo systemctl edit ollama
```

Add this service override:

```ini
[Service]
Environment="OLLAMA_HOST=0.0.0.0:11434"
```

Then apply it:

```bash
sudo systemctl daemon-reload
sudo systemctl restart ollama
curl http://127.0.0.1:11434/api/tags
```

From the NX, verify connectivity and launch perception. The current launch
defaults already use the AGX control-LAN address `192.168.123.99`:

```bash
curl http://192.168.123.99:11434/api/tags
ros2 launch om6dof_perception perception.launch.py \
  target_description:="red cup on the table"
```

Ollama has no authentication on this endpoint. Expose port `11434` only on the
trusted robot LAN or restrict it with a firewall.
