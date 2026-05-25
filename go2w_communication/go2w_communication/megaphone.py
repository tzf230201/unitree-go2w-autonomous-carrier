"""Send synthesized audio to the Go2W body speaker via the audiohub megaphone API.

Discovered through the community SDK (legion1581/go2_webrtc_connect): the
audiohub service accepts requests on /api/audiohub/request with these api_ids:

    4001  ENTER_MEGAPHONE     param: {}
    4003  UPLOAD_MEGAPHONE    param: {current_block_size, block_content (base64),
                                       current_block_index, total_block_number}
    4002  EXIT_MEGAPHONE      param: {}

The block_content is the WAV file (44.1 kHz mono 16-bit) base64-encoded, chunked
into 4 KB strings, sent sequentially with ~100 ms gap between chunks.
"""
from __future__ import annotations

import base64
import io
import json
import time
import wave

import audioop

import rclpy
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from unitree_api.msg import Request

REQUEST_TOPIC = "/api/audiohub/request"
ENTER_MEGAPHONE = 4001
EXIT_MEGAPHONE = 4002
UPLOAD_MEGAPHONE = 4003

TARGET_RATE = 44100
CHUNK_SIZE = 4096
INTER_CHUNK_DELAY = 0.1


def wav_from_pcm(raw_pcm: bytes, src_rate: int, channels: int = 1) -> bytes:
    """Resample mono int16 PCM to 44.1 kHz and wrap in a WAV blob."""
    if channels != 1:
        raise ValueError("Only mono input is supported")
    if src_rate != TARGET_RATE:
        raw_pcm, _ = audioop.ratecv(raw_pcm, 2, 1, src_rate, TARGET_RATE, None)
    out = io.BytesIO()
    with wave.open(out, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(TARGET_RATE)
        w.writeframes(raw_pcm)
    return out.getvalue()


class MegaphonePlayer:
    """Minimal client to push WAV audio through the Go2W speaker."""

    def __init__(self, node_name: str = "go2w_megaphone"):
        rclpy.init()
        self.node = rclpy.create_node(node_name)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub = self.node.create_publisher(Request, REQUEST_TOPIC, qos)
        # Allow DDS discovery to settle.
        time.sleep(0.4)

    def _call(self, api_id: int, params: dict):
        req = Request()
        req.header.identity.id = api_id
        req.header.identity.api_id = api_id
        req.parameter = json.dumps(params, ensure_ascii=True)
        self._pub.publish(req)

    def play_wav(self, wav_blob: bytes, tail_wait: float = 0.8):
        """Stream a WAV blob through the speaker and wait for it to finish."""
        b64 = base64.b64encode(wav_blob).decode("ascii")
        chunks = [b64[i : i + CHUNK_SIZE] for i in range(0, len(b64), CHUNK_SIZE)]

        self._call(ENTER_MEGAPHONE, {})
        time.sleep(0.1)

        for i, ch in enumerate(chunks, 1):
            self._call(
                UPLOAD_MEGAPHONE,
                {
                    "current_block_size": len(ch),
                    "block_content": ch,
                    "current_block_index": i,
                    "total_block_number": len(chunks),
                },
            )
            time.sleep(INTER_CHUNK_DELAY)

        # Approximate audio duration in seconds (16-bit mono).
        audio_seconds = len(wav_blob) / TARGET_RATE / 2
        time.sleep(max(1.5, audio_seconds + tail_wait))

        self._call(EXIT_MEGAPHONE, {})
        time.sleep(0.2)

    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()
