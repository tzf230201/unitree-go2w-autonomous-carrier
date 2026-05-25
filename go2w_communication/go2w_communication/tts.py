"""Wrap Piper TTS so callers get raw PCM (sample rate, mono int16)."""
from __future__ import annotations

import io
import wave
from pathlib import Path

from piper import PiperVoice

DEFAULT_VOICE = "/home/unitree/piper_voices/en_US-amy-medium.onnx"


class TtsEngine:
    def __init__(self, voice_path: str | Path = DEFAULT_VOICE):
        if not Path(voice_path).exists():
            raise FileNotFoundError(
                f"Piper voice model not found at {voice_path}. "
                "Run `wget` instructions from the README to download it."
            )
        self._voice = PiperVoice.load(str(voice_path))

    def synthesize(self, text: str) -> tuple[bytes, int]:
        """Return (raw int16 PCM bytes, sample_rate_hz)."""
        buf = io.BytesIO()
        with wave.open(buf, "wb") as wf:
            self._voice.synthesize_wav(text, wf)
        buf.seek(0)
        with wave.open(buf, "rb") as r:
            return r.readframes(r.getnframes()), r.getframerate()
