"""Streaming chat over WebRTC: speak sentences as the LLM produces them.

Compared to chat_stream (which uses the audiohub megaphone API over DDS), this
keeps a single WebRTC peer connection open and pushes PCM frames into a custom
AudioStreamTrack as Piper renders them. First audio leaves the robot within a
few hundred ms of the first sentence finishing.

Usage:
    ros2 run go2w_communication chat_webrtc "Tell me about the moon"
    ros2 run go2w_communication chat_webrtc
"""
from __future__ import annotations

import argparse
import asyncio
import json
import re
import sys
import time
from typing import Optional

import requests

from .tts import DEFAULT_VOICE, TtsEngine
from .webrtc_speaker import DEFAULT_ROBOT_IP, WebRTCSpeaker

OLLAMA_URL = "http://localhost:11434/api/generate"
DEFAULT_MODEL = "qwen2.5:7b"
DEFAULT_CONTEXT = 2048
SYSTEM_PROMPT = (
    "You are a friendly robot assistant. Reply in 1 to 3 SHORT independent "
    "sentences (each under 20 words), ending each with a period. Do not chain "
    "clauses with semicolons or commas — start a new sentence instead. "
    "Plain English only, suitable for text-to-speech. No markdown, no emojis, "
    "no lists."
)

_SENT_RE = re.compile(r'.+?(?:[.!?;][\'")\]]*(?=\s)|[.!?;][\'")\]]*$|\n)', re.DOTALL)


async def _stream_sentences(prompt: str, model: str, num_predict: int,
                            num_ctx: int):
    """Async generator yielding complete sentences as they arrive from Ollama."""
    payload = {
        "model": model,
        "prompt": prompt,
        "stream": True,
        "system": SYSTEM_PROMPT,
        "options": {"num_predict": num_predict, "num_ctx": num_ctx},
    }

    loop = asyncio.get_running_loop()
    # Use a thread for the blocking requests call; pipe tokens via a queue.
    queue: asyncio.Queue[Optional[str]] = asyncio.Queue()

    def worker():
        buf = ""
        try:
            with requests.post(OLLAMA_URL, json=payload, stream=True, timeout=180) as r:
                r.raise_for_status()
                for line in r.iter_lines(decode_unicode=True):
                    if not line:
                        continue
                    obj = json.loads(line)
                    tok = obj.get("response", "")
                    buf += tok
                    while True:
                        m = _SENT_RE.match(buf)
                        if not m:
                            break
                        sent = m.group(0).strip()
                        buf = buf[m.end():]
                        if sent:
                            asyncio.run_coroutine_threadsafe(queue.put(sent), loop)
                    if obj.get("done"):
                        break
            if buf.strip():
                asyncio.run_coroutine_threadsafe(queue.put(buf.strip()), loop)
        finally:
            asyncio.run_coroutine_threadsafe(queue.put(None), loop)

    loop.run_in_executor(None, worker)

    while True:
        item = await queue.get()
        if item is None:
            return
        yield item


async def _synthesize(tts: TtsEngine, sentence: str):
    """Run blocking Piper synth in a thread; return (pcm_bytes, rate)."""
    loop = asyncio.get_running_loop()
    return await loop.run_in_executor(None, tts.synthesize, sentence)


async def run_chat(prompt: str, model: str, tts: TtsEngine,
                   speaker: WebRTCSpeaker, num_predict: int = 120,
                   num_ctx: int = DEFAULT_CONTEXT):
    print(f"\n[user] {prompt}")
    t_start = time.time()
    first_audio_at: Optional[float] = None

    async for sentence in _stream_sentences(prompt, model, num_predict, num_ctx):
        pcm, rate = await _synthesize(tts, sentence)
        await speaker.speak_pcm(pcm, rate)
        if first_audio_at is None:
            first_audio_at = time.time() - t_start
            print(f"[stream] first audio queued in {first_audio_at:.2f}s")
        print(f"[say] {sentence}")

    await speaker.wait_drained()
    print(f"[done] {time.time() - t_start:.2f}s total")


async def amain(args: argparse.Namespace):
    # Connect WebRTC first — Piper's onnxruntime import is heavy and can
    # interfere with the data-channel handshake timing if loaded beforehand.
    print(f"Connecting WebRTC to {args.robot_ip}...")
    speaker = WebRTCSpeaker(ip=args.robot_ip)
    await speaker.connect()
    print("WebRTC connected.")
    print("Loading Piper voice...")
    tts = TtsEngine(args.voice)

    try:
        if args.prompt:
            await run_chat(" ".join(args.prompt), args.model, tts, speaker,
                           num_predict=args.num_predict)
        else:
            print("Interactive mode. Type a question, Ctrl+C / Ctrl+D to quit.")
            loop = asyncio.get_running_loop()
            while True:
                try:
                    q = await loop.run_in_executor(None, lambda: input("\n> ").strip())
                except (EOFError, KeyboardInterrupt):
                    print()
                    break
                if not q:
                    continue
                await run_chat(q, args.model, tts, speaker,
                               num_predict=args.num_predict)
    finally:
        await speaker.disconnect()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Streaming chat over WebRTC to the Go2W body speaker."
    )
    parser.add_argument("prompt", nargs="*", help="prompt; omit for interactive mode")
    parser.add_argument("--model", default=DEFAULT_MODEL, help="Ollama model tag")
    parser.add_argument("--voice", default=DEFAULT_VOICE, help="Piper voice .onnx path")
    parser.add_argument("--robot-ip", default=DEFAULT_ROBOT_IP, help="Go2W IP address")
    parser.add_argument("--num-predict", type=int, default=120,
                        help="max tokens in the LLM reply")
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    try:
        asyncio.run(amain(args))
    except KeyboardInterrupt:
        print("\nInterrupted.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
