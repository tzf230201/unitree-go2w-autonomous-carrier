"""Ask a question, answered by a local Ollama model, spoken via the Go2W speaker.

Usage:
    ros2 run go2w_communication chat "What can you do?"          # one-shot
    ros2 run go2w_communication chat                              # interactive
"""
from __future__ import annotations

import argparse
import sys
import time

import requests

from .megaphone import MegaphonePlayer, wav_from_pcm
from .tts import DEFAULT_VOICE, TtsEngine

OLLAMA_URL = "http://localhost:11434/api/generate"
DEFAULT_MODEL = "qwen2.5:7b"
DEFAULT_CONTEXT = 2048   # smaller context keeps memory comfortable on Orin NX
SYSTEM_PROMPT = (
    "You are a friendly robot assistant. Reply in ONE short sentence "
    "(under 25 words) using plain English suitable for text-to-speech. "
    "No markdown, no emojis, no lists. Keep it punchy."
)


def ask_ollama(prompt: str, model: str, num_predict: int = 60,
               num_ctx: int = DEFAULT_CONTEXT) -> str:
    payload = {
        "model": model,
        "prompt": prompt,
        "stream": False,
        "system": SYSTEM_PROMPT,
        "options": {"num_predict": num_predict, "num_ctx": num_ctx},
    }
    r = requests.post(OLLAMA_URL, json=payload, timeout=180)
    r.raise_for_status()
    return r.json()["response"].strip()


def run_turn(prompt: str, model: str, tts: TtsEngine, player: MegaphonePlayer):
    print(f"\n[user]   {prompt}")
    t0 = time.time()
    answer = ask_ollama(prompt, model)
    print(f"[ollama] ({time.time() - t0:.1f}s) {answer}")

    t1 = time.time()
    pcm, rate = tts.synthesize(answer)
    wav = wav_from_pcm(pcm, rate)
    print(f"[tts]    ({time.time() - t1:.1f}s) {len(wav)} bytes WAV")

    t2 = time.time()
    player.play_wav(wav)
    print(f"[speak]  ({time.time() - t2:.1f}s) done")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Chat with Ollama through the Go2W speaker.")
    parser.add_argument("prompt", nargs="*", help="prompt; omit for interactive mode")
    parser.add_argument("--model", default=DEFAULT_MODEL, help="Ollama model tag")
    parser.add_argument("--voice", default=DEFAULT_VOICE, help="Piper voice .onnx path")
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    print("Loading Piper voice...")
    tts = TtsEngine(args.voice)
    player = MegaphonePlayer()

    try:
        if args.prompt:
            run_turn(" ".join(args.prompt), args.model, tts, player)
        else:
            print("Interactive mode. Type your question, Ctrl+C / Ctrl+D to quit.")
            while True:
                try:
                    q = input("\n> ").strip()
                except (EOFError, KeyboardInterrupt):
                    print()
                    break
                if not q:
                    continue
                run_turn(q, args.model, tts, player)
    finally:
        player.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
