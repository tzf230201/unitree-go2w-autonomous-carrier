# go2w_communication

Voice interaction for the Unitree Go2W body speaker.

Two entry points:

| Command | What it does |
|---|---|
| `ros2 run go2w_communication speak "text"` | Speak the given text through the robot speaker via Piper TTS. |
| `ros2 run go2w_communication chat "question"` | Ask Ollama (local LLM), speak the answer through the robot speaker. Omit the question for interactive mode. |

The package talks to the **audiohub megaphone** API over DDS
(`/api/audiohub/request`, api_id 4001 / 4002 / 4003) — the same API the Unitree
mobile app uses for the "megaphone" feature. Constants were sourced from the
community SDK [`legion1581/go2_webrtc_connect`](https://github.com/legion1581/go2_webrtc_connect).

---

## Install (one-time setup)

The package itself is pure Python, but it depends on three pieces that have to
be set up outside `colcon`:

1. **Unitree ROS 2 / DDS environment** — `~/unitree_ros2` already configured.
2. **A Python environment with Piper TTS + ROS message bindings** (a venv
   sharing system site-packages works fine).
3. **A local Ollama install** (only for the `chat` command).

### 1. System packages

```bash
sudo apt update
sudo apt install -y python3.10-venv python3-pip
```

### 2. Python venv with system access

The venv shares system site-packages so `rclpy`, `unitree_api`, `numpy`, etc.
remain importable.

```bash
python3 -m venv --system-site-packages ~/ggcnn_env
source ~/ggcnn_env/bin/activate
pip install --upgrade pip
pip install piper-tts requests
# Keep numpy < 2 so it stays binary-compatible with the system cv2.
pip install "numpy<2"
```

### 3. Download a Piper voice

```bash
mkdir -p ~/piper_voices
cd ~/piper_voices
wget https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/amy/medium/en_US-amy-medium.onnx
wget https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/amy/medium/en_US-amy-medium.onnx.json
```

To use a different voice, pass `--voice /path/to/voice.onnx` on the command
line. Voice list: <https://huggingface.co/rhasspy/piper-voices>.

### 4. Install Ollama (only for `chat`)

```bash
curl -fsSL https://ollama.com/install.sh | sh
ollama pull qwen2.5:7b          # default model — solid Q&A on Orin NX 16GB
# Optional smaller/larger options:
#   ollama pull llama3.2:3b     # ~2 GB, fast (~10 tok/s) but limited reasoning
#   ollama pull llama3.1:8b     # ~5 GB, ~8 tok/s
#   ollama pull qwen2.5:14b     # ~9 GB, ~3-5 tok/s, sharper but mepet memory
```

On Jetson Orin NX with CUDA 12.6 the installer auto-builds the JetPack 6 CUDA
backend; verify with:

```bash
journalctl -u ollama --no-pager -n 30 | grep -E 'compute|library=CUDA'
# expected: library=CUDA compute=8.7 name=Orin
```

If a 7B+ model fails to load with `cudaMalloc failed: out of memory`, free the
page cache (Jetson uses unified memory, so file cache competes with the GPU):

```bash
sudo sh -c 'echo 3 > /proc/sys/vm/drop_caches'
```

### 5. Build the ROS 2 package

```bash
cd ~/ros2_ws
colcon build --packages-select go2w_communication --symlink-install
# ament_python hard-codes /usr/bin/python3 in entry-point shebangs, so the
# built `chat` / `speak` scripts can't find Piper. Patch them to the venv:
bash src/unitree-go2w-autonomous-carrier/go2w_communication/scripts/fix_shebang.sh
```

Rerun `fix_shebang.sh` after every rebuild. If your venv lives elsewhere:

```bash
VENV_PY=/path/to/venv/bin/python bash .../fix_shebang.sh
```

### 6. Source the environment in every new shell

```bash
source ~/unitree_ros2/setup.sh        # ROS 2 + Cyclone DDS bound to enP8p1s0
source ~/ros2_ws/install/setup.bash   # this package
source ~/ggcnn_env/bin/activate       # Piper + requests
```

---

## Usage

### Just speak some text

```bash
ros2 run go2w_communication speak "Hello, I am the Unitree robot."

# or pipe stdin
echo "Hello from the pipeline" | ros2 run go2w_communication speak
```

### Chat with Ollama, answer is spoken

```bash
ros2 run go2w_communication chat "What can you do as a robot?"

# Interactive
ros2 run go2w_communication chat
> What is your favourite color?
> /quit                              # Ctrl+C or Ctrl+D also works
```

### Tuning

- **Model** — `--model qwen2.5:7b` (default) is a good balance for Orin NX.
  Drop to `llama3.2:3b` for speed, jump to `qwen2.5:14b` for sharper answers
  (`sudo sh -c 'echo 3 > /proc/sys/vm/drop_caches'` first if it OOMs).
- **Response length** — edit `chat.py:num_predict` (default 120 tokens).
- **Voice** — pass `--voice /path/to/voice.onnx`.
- **Playback speed** — `INTER_CHUNK_DELAY` in `megaphone.py` defaults to 100 ms
  per 4 KB chunk; reducing it to 0.05 roughly halves the upload time on a
  stable Wi-Fi link. Lower than that risks dropping chunks.

---

## How it works

```
+----------------+  text  +--------+  PCM 22 kHz  +--------+  WAV 44.1 kHz
|  Ollama HTTP   |------->|  Piper |------------->|  ratecv|----+
| 127.0.0.1:11434|        |  TTS   |              |        |    |
+----------------+        +--------+              +--------+    |
                                                                v
                                              base64, 4 KB chunks, 100 ms apart
                                                                |
                       +----------------+   /api/audiohub/      v
                       | Go2W audiohub  |<-- request, api_id  +-----+
                       |  (firmware)    |   4001 / 4003 / 4002| DDS |
                       +----------------+                     +-----+
                              |
                              v   (robot body speaker)
```

The audiohub treats megaphone uploads as a stream of base64-encoded WAV chunks.
The full WAV file (header + PCM) is sent across `UPLOAD_MEGAPHONE` requests in
4 KB chunks. The firmware concatenates them on its side and plays the resulting
buffer once `EXIT_MEGAPHONE` is called (or once playback completes naturally).

---

## Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `ImportError: rclpy` | venv was created without `--system-site-packages`; recreate it. |
| `ModuleNotFoundError: unitree_api` | Forgot `source ~/unitree_ros2/setup.sh`. |
| `FileNotFoundError: en_US-amy-medium.onnx` | Step 3 not run, or wrong path; pass `--voice`. |
| No sound but everything returns code 0 | Robot speaker volume might be 0 — set via the Unitree mobile app, or call the Go2 `vui` `SetVolume` service. |
| Numpy 2.x / cv2 ABI error | Run `pip install "numpy<2"` inside the venv. |
| Ollama connection refused | `systemctl status ollama`; service listens on `127.0.0.1:11434`. |
