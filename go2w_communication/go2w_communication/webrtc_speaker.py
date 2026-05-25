"""Push PCM audio to the Go2W body speaker over WebRTC.

Uses legion1581/go2_webrtc_connect to set up the WebRTC peer connection, then
attaches a custom AudioStreamTrack that pulls 20 ms s16 PCM frames from an
asyncio queue.  Frames are produced from Piper TTS output (any sample rate,
resampled to 48 kHz mono).

The connection stays open so that multiple sentences play back-to-back without
the megaphone API's per-request overhead.
"""
from __future__ import annotations

import asyncio
import audioop
import fractions
import time
from typing import Optional

# IMPORTANT: import unitree_webrtc_connect BEFORE aiortc/av — its __init__
# monkey-patches aioice.Connection. If aiortc has already imported aioice,
# it caches a stale reference and the data channel never opens.
from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection,
    WebRTCConnectionMethod,
)

import av  # noqa: E402
from aiortc import MediaStreamTrack  # noqa: E402

DEFAULT_ROBOT_IP = "192.168.123.161"
RTC_RATE = 48000           # WebRTC standard sample rate
SAMPLES_PER_FRAME = 960    # 20 ms at 48 kHz mono
BYTES_PER_FRAME = SAMPLES_PER_FRAME * 2  # s16 mono


class _PCMQueueTrack(MediaStreamTrack):
    """Audio track whose frames come from a queue of 20 ms PCM chunks."""

    kind = "audio"

    def __init__(self):
        super().__init__()
        self._queue: asyncio.Queue[bytes] = asyncio.Queue()
        self._pts = 0
        self._time_base = fractions.Fraction(1, RTC_RATE)
        # Real-time pacing: emit at most one 20 ms frame per 20 ms wall-clock.
        self._next_emit_at: float | None = None
        self._frames_emitted = 0

    async def push_pcm48k(self, pcm: bytes):
        """Enqueue PCM (48 kHz, mono, s16) as 20 ms frames. Pads the last frame."""
        for i in range(0, len(pcm), BYTES_PER_FRAME):
            chunk = pcm[i : i + BYTES_PER_FRAME]
            if len(chunk) < BYTES_PER_FRAME:
                chunk = chunk + b"\x00" * (BYTES_PER_FRAME - len(chunk))
            await self._queue.put(chunk)

    def pending_seconds(self) -> float:
        return self._queue.qsize() * SAMPLES_PER_FRAME / RTC_RATE

    async def recv(self) -> av.AudioFrame:
        chunk = await self._queue.get()
        frame = av.AudioFrame(format="s16", layout="mono", samples=SAMPLES_PER_FRAME)
        frame.planes[0].update(chunk)
        frame.sample_rate = RTC_RATE
        frame.pts = self._pts
        frame.time_base = self._time_base
        self._pts += SAMPLES_PER_FRAME
        self._frames_emitted += 1
        return frame


def resample_to_48k(pcm: bytes, src_rate: int) -> bytes:
    """Convert mono s16 PCM at any rate to 48 kHz."""
    if src_rate == RTC_RATE:
        return pcm
    out, _ = audioop.ratecv(pcm, 2, 1, src_rate, RTC_RATE, None)
    return out


class WebRTCSpeaker:
    """Open a persistent WebRTC session to the robot and stream PCM into it."""

    def __init__(self, ip: str = DEFAULT_ROBOT_IP):
        self.ip = ip
        self.conn: Optional[UnitreeWebRTCConnection] = None
        self.track = _PCMQueueTrack()

    async def connect(self):
        self.conn = UnitreeWebRTCConnection(
            WebRTCConnectionMethod.LocalSTA, ip=self.ip
        )
        await self.conn.connect()
        # Reuse the pre-allocated audio transceiver instead of addTrack(),
        # which would create a new un-negotiated transceiver. replaceTrack
        # swaps the sender's source without requiring re-negotiation.
        attached = False
        for tr in self.conn.pc.getTransceivers():
            if tr.kind == "audio":
                tr.sender.replaceTrack(self.track)
                attached = True
                break
        if not attached:
            # Fallback (shouldn't happen given SDK pre-adds the transceiver)
            self.conn.pc.addTrack(self.track)
        # Tell the robot firmware to route audio frames to the speaker.
        self.conn.datachannel.switchAudioChannel(True)
        await asyncio.sleep(0.5)

    async def speak_pcm(self, pcm: bytes, src_rate: int):
        """Queue one utterance for playback. Returns immediately."""
        pcm48 = resample_to_48k(pcm, src_rate)
        await self.track.push_pcm48k(pcm48)

    async def wait_drained(self, extra: float = 5.0):
        """Block until queued audio has been emitted by the encoder.

        The encoder typically drains the queue much faster than real-time
        playback. After the queue empties, give the robot generous time to
        play the buffered audio out (network + jitter buffer + speaker).
        """
        while self.track.pending_seconds() > 0:
            await asyncio.sleep(0.1)
        await asyncio.sleep(extra)

    async def disconnect(self):
        if self.conn is not None:
            try:
                self.conn.datachannel.switchAudioChannel(False)
            except Exception:
                pass
            if hasattr(self.conn, "disconnect"):
                try:
                    await self.conn.disconnect()
                except Exception:
                    pass
