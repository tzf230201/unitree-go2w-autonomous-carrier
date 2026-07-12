# go2w_communication

Send text to the Unitree Go2W body speaker — local LLM, local TTS, or just a
prerecorded phrase. Three entry points:

| Command | What it does |
|---|---|
| `ros2 run go2w_communication speak "text"` | Speak the given text through the robot speaker via local TTS. |
| `ros2 run go2w_communication chat_stream "question"` | Ask a local Ollama LLM and stream the answer sentence-by-sentence to the speaker. **Recommended.** |
| `ros2 run go2w_communication chat "question"` | Batch variant (waits for full LLM reply before speaking). Older / simpler. |
| `ros2 run go2w_communication chat_webrtc "question"` | Experimental WebRTC variant. Sometimes works, sometimes silent — left in for future investigation. |

Audio reaches the speaker via the **audiohub megaphone** API on
`/api/audiohub/request` (api_ids 4001 / 4003 / 4002). These IDs are not
documented in Unitree's SDK — they were taken from the community project
[`legion1581/go2_webrtc_connect`](https://github.com/legion1581/go2_webrtc_connect).

Unitree's official "Benben" cloud assistant (`/api/gpt/request`) is *not* used:
it returns `state: Error1, "Temporary failure in name resolution"` on this rig
because the robot can't reach Unitree's cloud endpoint.

---

## Install (one-time)

The ROS 2 package is pure Python. Three external pieces are required:

1. **ROS 2 / DDS env** — `~/unitree_ros2` already configured for CycloneDDS on `enP8p1s0`.
2. **A Python venv** that sees system site-packages (so it can import `rclpy` and `unitree_api`) plus Piper, Kokoro, and Ollama HTTP deps.
3. **A local Ollama install** with at least one model (only needed for the `chat*` commands).

### 1. System packages

```bash
sudo apt update
sudo apt install -y python3.10-venv python3-pip ffmpeg
```

### 2. Python venv with system access

```bash
python3 -m venv --system-site-packages ~/ggcnn_env
source ~/ggcnn_env/bin/activate
pip install --upgrade pip
pip install "numpy<2"           # keep ABI-compatible with system cv2
pip install requests
pip install piper-tts            # fast TTS, decent quality
pip install kokoro               # slower but more natural — default for chat_stream
```

Optional WebRTC bits (only for `chat_webrtc`):

```bash
sudo apt install -y portaudio19-dev libsrtp2-dev libopus-dev libvpx-dev \
                    libavdevice-dev libavfilter-dev libswresample-dev \
                    libswscale-dev libavutil-dev
pip install unitree-webrtc-connect aiortc
```

### 3. Download Piper voice

Piper still runs the `speak` and `chat` commands. Default voice is `ljspeech-high`:

```bash
mkdir -p ~/piper_voices
cd ~/piper_voices
wget https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/ljspeech/high/en_US-ljspeech-high.onnx
wget https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/ljspeech/high/en_US-ljspeech-high.onnx.json
```

Kokoro auto-downloads `hexgrad/Kokoro-82M` on first use (~325 MB cached under
`~/.cache/huggingface`).

### 4. Install Ollama (only for chat / chat_stream / chat_webrtc)

```bash
curl -fsSL https://ollama.com/install.sh | sh
ollama pull qwen2.5:7b           # default (smarter)
ollama pull llama3.2:3b          # automatic fallback when qwen OOMs
```

Verify the Jetson 6 CUDA backend is loaded:

```bash
journalctl -u ollama --no-pager -n 30 | grep -E 'compute|library=CUDA'
# expected: library=CUDA compute=8.7 name=Orin libdirs=ollama,cuda_jetpack6
```

Tight on RAM — Jetson Orin NX has 15 GB unified between CPU & GPU. Free the
page cache before launching a 7B model:

```bash
sudo sh -c 'echo 3 > /proc/sys/vm/drop_caches'
```

`chat_stream` will auto-fall-back to `llama3.2:3b` when `qwen2.5:7b` OOMs, so
this is optional now.

### 5. Build the ROS 2 package

```bash
cd ~/ros2_ws
colcon build --packages-select go2w_communication --symlink-install
bash src/unitree-go2w-autonomous-carrier/go2w_communication/scripts/fix_shebang.sh
```

`fix_shebang.sh` rewrites the `#!/usr/bin/python3` shebang in the installed
entry-point scripts to point at the venv's Python. ament_python hardcodes the
system shebang, but Piper/Kokoro/aiortc live in `~/ggcnn_env`. **Rerun
`fix_shebang.sh` after every rebuild.**

### 6. Source in every new shell

```bash
source ~/unitree_ros2/setup.sh        # ROS 2 + Cyclone DDS bound to enP8p1s0
source ~/ros2_ws/install/setup.bash   # this package
source ~/ggcnn_env/bin/activate       # Piper / Kokoro / requests
```

---

## Usage

### Just speak some text

```bash
ros2 run go2w_communication speak "Hello, I am the Unitree robot."
echo "Audio test" | ros2 run go2w_communication speak
```

### Streaming chat (recommended)

```bash
ros2 run go2w_communication chat_stream "Tell me about Mars."

# Interactive
ros2 run go2w_communication chat_stream
> What time is it?
> What's the weather like?
> ^D
```

### Batch chat (older, slower)

```bash
ros2 run go2w_communication chat "What can you do?"
```

### Tuning

| Flag | What it does |
|---|---|
| `--model llama3.2:3b` | Force a specific LLM (skip the qwen→llama fallback). |
| `--engine piper` | Use Piper TTS instead of Kokoro (~3x faster, less natural). |
| `--voice af_bella` | Choose a different Kokoro voice (`af_heart` is default; see the [Kokoro voice list](https://huggingface.co/hexgrad/Kokoro-82M)). |
| `--voice /path/to/foo.onnx` | When `--engine piper`, point at any Piper `.onnx`. |

### Latency anatomy of `chat_stream`

```
+-------------+   stream tokens   +----------+   sentence   +------------+   WAV
|   Ollama    | --- HTTP /api --> | sentence | -----------> |  Kokoro /  | -------> +-------------+
| qwen2.5:7b  |     (stream=True) | splitter |              |   Piper    |          | Megaphone   | --> robot
| (auto-fall- |                   +----------+              | TTS engine |          | (api 4001/  |     speaker
|  back 3B    |                                             +------------+          |   4003/4002)|
|  on OOM)    |                                                                     +-------------+
+-------------+
```

First audio leaves the robot when the **first** sentence is rendered, not when
the LLM finishes the whole reply.

---

## How it works

The audio path uses Unitree's undocumented audiohub megaphone API. Per
utterance:

1. `ENTER_MEGAPHONE` (api_id 4001, empty params) — robot enters streaming-input mode.
2. `UPLOAD_MEGAPHONE` (api_id 4003) × N — base64 of the WAV file chopped into 4 KB chunks, each request includes `{current_block_size, block_content, current_block_index, total_block_number}`. The script paces chunks ~`INTER_CHUNK_DELAY` (default 0.05 s) apart — faster causes firmware to drop frames (audible pitch-up or silence).
3. `EXIT_MEGAPHONE` (api_id 4002) — close the session, robot finishes playing.

The WAV blob is mono 44.1 kHz int16. Piper renders at 22 kHz, Kokoro at 24 kHz;
`megaphone.wav_from_pcm()` resamples to 44.1 kHz before sending.

For `chat_stream`, the same megaphone session stays open across all sentences
in one reply — chunks are pushed back-to-back, the robot plays as it receives.

---

## Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `ModuleNotFoundError: rclpy` | Forgot `source ~/unitree_ros2/setup.sh`. |
| `ModuleNotFoundError: piper` / `kokoro` | Forgot `source ~/ggcnn_env/bin/activate`. |
| Console says success but no sound | Robot speaker volume might be 0 — change via Unitree mobile app or the `vui` service. |
| `cudaMalloc failed: out of memory` from ollama | The page cache competes with the GPU on the Jetson's unified RAM. `sudo sh -c 'echo 3 > /proc/sys/vm/drop_caches'` — or let auto-fallback take over. |
| Speech sounds sped up / chipmunk | `INTER_CHUNK_DELAY` is too low; back off to 0.1. |
| `chat_webrtc` connects but silent | Known: the SDK pre-allocates a sendrecv audio transceiver during `connect()`; we patch the existing track via `replaceTrack`, but the firmware audio path is flaky after sessions. Restart the robot or use `chat_stream`. |
| ROS-vs-Python ABI errors (numpy 2.x cv2) | Pin numpy < 2 inside the venv. |
| `aiortc` import breaks Unitree WebRTC connection | The SDK monkey-patches `aioice.Connection` on import; you must import `unitree_webrtc_connect.webrtc_driver` *before* `aiortc`. `webrtc_speaker.py` does this in the right order; user code that mixes them needs to follow the same rule. |

---

## File layout

```
go2w_communication/
├── package.xml
├── setup.py
├── setup.cfg
├── README.md
├── resource/go2w_communication
├── scripts/
│   └── fix_shebang.sh           # patches installed entry-point shebangs
└── go2w_communication/
    ├── __init__.py
    ├── megaphone.py             # audiohub megaphone DDS client (4001/4003/4002)
    ├── tts.py                   # TtsEngine (Piper) + KokoroEngine
    ├── speak.py                 # entry: speak
    ├── chat.py                  # entry: chat (batch)
    ├── chat_stream.py           # entry: chat_stream (streaming, recommended)
    ├── chat_webrtc.py           # entry: chat_webrtc (experimental)
    └── webrtc_speaker.py        # WebRTC custom audio track
```
