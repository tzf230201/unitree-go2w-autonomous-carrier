"""Speak arbitrary text through the Go2W body speaker.

Usage:
    ros2 run go2w_communication speak "Hello, this is the robot."
"""
from __future__ import annotations

import argparse
import sys

from .megaphone import MegaphonePlayer, wav_from_pcm
from .tts import DEFAULT_VOICE, TtsEngine


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Speak text through the Go2W speaker.")
    parser.add_argument("text", nargs="*", help="text to speak; reads stdin if empty")
    parser.add_argument("--voice", default=DEFAULT_VOICE, help="Piper voice .onnx path")
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    text = " ".join(args.text).strip() or sys.stdin.read().strip()
    if not text:
        print("No text provided.", file=sys.stderr)
        return 1

    print(f'[tts] "{text}"')
    tts = TtsEngine(args.voice)
    pcm, rate = tts.synthesize(text)
    wav = wav_from_pcm(pcm, rate)
    print(f"[tts] {len(wav)} bytes WAV @ 44100 Hz mono")

    player = MegaphonePlayer()
    try:
        player.play_wav(wav)
        print("[speak] done")
    finally:
        player.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
