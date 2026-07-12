"""TTS engines: Piper (fast) and Kokoro (more natural). Common interface."""
from __future__ import annotations

import io
import wave
from pathlib import Path

DEFAULT_VOICE = "/home/unitree/piper_voices/en_US-ljspeech-high.onnx"
KOKORO_VOICE = "af_heart"   # Kokoro default speaker


class TtsEngine:
    """Piper TTS — fast (~0.5x rtf on CPU), good quality."""

    def __init__(self, voice_path: str | Path = DEFAULT_VOICE):
        from piper import PiperVoice
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


class KokoroEngine:
    """Kokoro-82M TTS — more natural prosody, ~1.5-2x rtf on CPU.

    Heavier than Piper but produces noticeably more human-like speech.
    Lazy-loads the model on first synthesize() call.
    """

    def __init__(self, voice: str = KOKORO_VOICE, lang_code: str = "a"):
        # Defer import — kokoro pulls heavy deps (spacy, transformers).
        from kokoro import KPipeline
        self._voice = voice
        self._pipeline = KPipeline(lang_code=lang_code, device="cpu")

    def synthesize(self, text: str) -> tuple[bytes, int]:
        import numpy as np
        segments = list(self._pipeline(text, voice=self._voice))
        if not segments:
            return b"", 24000
        audio = np.concatenate([s[2] for s in segments])
        audio16 = (np.clip(audio, -1.0, 1.0) * 32767).astype(np.int16)
        return audio16.tobytes(), 24000


def load_engine(name: str = "piper", **kwargs) -> object:
    """Pick a TTS engine by name. name in {'piper', 'kokoro'}."""
    if name == "kokoro":
        return KokoroEngine(**kwargs)
    return TtsEngine(**kwargs)
