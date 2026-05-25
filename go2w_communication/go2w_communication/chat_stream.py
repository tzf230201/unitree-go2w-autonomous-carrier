"""Streaming chat: speak sentences as the LLM produces them.

Pipeline (three concurrent stages):

    Ollama (stream=True)  ->  sentence buffer  ->  TTS worker  ->  speaker
            tokens               on '.','!','?'        WAV          megaphone

The first audio leaves the robot ~as soon as the LLM finishes its first
sentence, while later sentences are still being generated and synthesized.

Usage:
    ros2 run go2w_communication chat_stream "What is the moon?"
    ros2 run go2w_communication chat_stream
"""
from __future__ import annotations

import argparse
import json
import queue
import re
import sys
import threading
import time

import requests

from .megaphone import MegaphonePlayer, wav_from_pcm
from .tts import DEFAULT_VOICE, KokoroEngine, TtsEngine, load_engine

OLLAMA_URL = "http://localhost:11434/api/generate"
DEFAULT_MODEL = "qwen2.5:7b"
FALLBACK_MODEL = "llama3.2:3b"   # used if DEFAULT_MODEL OOMs
DEFAULT_CONTEXT = 2048
SYSTEM_PROMPT = (
    "You are a friendly robot assistant. Reply in 1 to 3 short sentences using "
    "plain English suitable for text-to-speech. End each sentence with proper "
    "punctuation. No markdown, no emojis, no lists."
)

# A "complete sentence" ends in . ! ? possibly followed by ) " ' and then
# whitespace or end-of-buffer.  Also flush on newline.
_SENT_RE = re.compile(r'.+?(?:[.!?][\'")\]]*(?=\s)|[.!?][\'")\]]*$|\n)', re.DOTALL)
_END_DONE = object()


def _try_stream_one(prompt: str, model: str, out_q: queue.Queue,
                    num_predict: int, num_ctx: int):
    """Stream from a single Ollama model. Returns (full_text, error_str or None).

    On the first chunk the response is fully readable, so OOM/500 errors are
    detected before any sentences are emitted — making fallback safe.
    """
    payload = {
        "model": model,
        "prompt": prompt,
        "stream": True,
        "system": SYSTEM_PROMPT,
        "options": {"num_predict": num_predict, "num_ctx": num_ctx},
    }
    buf = ""
    full_text = []
    with requests.post(OLLAMA_URL, json=payload, stream=True, timeout=180) as r:
        if r.status_code != 200:
            body = r.text[:500]
            return "", f"HTTP {r.status_code}: {body}"
        for line in r.iter_lines(decode_unicode=True):
            if not line:
                continue
            obj = json.loads(line)
            if obj.get("error"):
                return "".join(full_text), obj["error"]
            tok = obj.get("response", "")
            buf += tok
            full_text.append(tok)
            while True:
                m = _SENT_RE.match(buf)
                if not m:
                    break
                sent = m.group(0).strip()
                buf = buf[m.end():]
                if sent:
                    out_q.put(sent)
            if obj.get("done"):
                break
    if buf.strip():
        out_q.put(buf.strip())
    return "".join(full_text).strip(), None


def _stream_ollama(prompt: str, model: str, out_q: queue.Queue,
                   num_predict: int = 120, num_ctx: int = DEFAULT_CONTEXT,
                   fallback: str | None = FALLBACK_MODEL):
    """Stream tokens; if `model` OOMs before producing output, retry on `fallback`."""
    text = ""
    try:
        text, err = _try_stream_one(prompt, model, out_q, num_predict, num_ctx)
        if err and not text and fallback and fallback != model:
            print(f"[llm] {model} failed ({err[:80]}); falling back to {fallback}")
            text, err2 = _try_stream_one(prompt, fallback, out_q, num_predict, num_ctx)
            if err2:
                print(f"[llm] fallback {fallback} also failed: {err2[:120]}")
        elif err:
            print(f"[llm] error after partial output: {err[:120]}")
    finally:
        out_q.put(_END_DONE)
    return text


def _tts_worker(in_q: queue.Queue, out_q: queue.Queue, tts: TtsEngine):
    """Convert sentences to WAV blobs and forward to player."""
    while True:
        item = in_q.get()
        if item is _END_DONE:
            out_q.put(_END_DONE)
            return
        pcm, rate = tts.synthesize(item)
        wav = wav_from_pcm(pcm, rate)
        out_q.put((item, wav))


def run_stream(prompt: str, model: str, tts: TtsEngine, player: MegaphonePlayer,
               num_predict: int = 120):
    print(f"\n[user]   {prompt}")
    sent_q: queue.Queue = queue.Queue()
    play_q: queue.Queue = queue.Queue()

    full_text_holder: list[str] = []

    def llm_thread():
        text = _stream_ollama(prompt, model, sent_q, num_predict=num_predict)
        full_text_holder.append(text)

    def tts_thread():
        _tts_worker(sent_q, play_q, tts)

    t_llm = threading.Thread(target=llm_thread, daemon=True)
    t_tts = threading.Thread(target=tts_thread, daemon=True)
    t_llm.start()
    t_tts.start()

    t_start = time.time()
    first_audio_at = None
    session_open = False
    audio_finished_at = t_start  # wall-clock time the last queued sentence ends
    total_audio = 0.0

    while True:
        item = play_q.get()
        if item is _END_DONE:
            break
        sent, wav = item

        if not session_open:
            player.stream_open()
            session_open = True

        if first_audio_at is None:
            first_audio_at = time.time() - t_start
            print(f"[stream] first sentence in {first_audio_at:.2f}s")
        print(f"[say] {sent}")

        # Send chunks back-to-back, no per-sentence playback wait.
        dur = player.stream_send(wav)
        total_audio += dur
        # Track when audio will finish playing in wall-clock time.
        now = time.time()
        if now > audio_finished_at:
            audio_finished_at = now
        audio_finished_at += dur

    if session_open:
        remaining = max(0.0, audio_finished_at - time.time())
        player.stream_close(remaining_seconds=remaining)

    t_llm.join()
    t_tts.join()

    total = time.time() - t_start
    if first_audio_at is None:
        print(f"[done] {total:.2f}s total — no audio produced "
              "(LLM or TTS error?)")
    else:
        print(f"[done] {total:.2f}s total, first audio at "
              f"{first_audio_at:.2f}s")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Streaming chat: speak sentences as Ollama produces them."
    )
    parser.add_argument("prompt", nargs="*", help="prompt; omit for interactive mode")
    parser.add_argument("--model", default=DEFAULT_MODEL, help="Ollama model tag")
    parser.add_argument("--engine", choices=["piper", "kokoro"], default="kokoro",
                        help="TTS engine (kokoro = more natural, piper = faster)")
    parser.add_argument("--voice", default=None,
                        help="Voice file (Piper .onnx path) or voice name (Kokoro)")
    parser.add_argument("--num-predict", type=int, default=120,
                        help="max tokens in the LLM reply")
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    if args.engine == "kokoro":
        voice = args.voice or "af_heart"
        print(f"Loading Kokoro voice ({voice}) — first call downloads model...")
        tts = KokoroEngine(voice=voice)
    else:
        voice = args.voice or DEFAULT_VOICE
        print(f"Loading Piper voice ({voice})...")
        tts = TtsEngine(voice)
    player = MegaphonePlayer(node_name="go2w_chat_stream")

    try:
        if args.prompt:
            run_stream(" ".join(args.prompt), args.model, tts, player,
                       num_predict=args.num_predict)
        else:
            print("Interactive mode. Type a question, Ctrl+C / Ctrl+D to quit.")
            while True:
                try:
                    q = input("\n> ").strip()
                except (EOFError, KeyboardInterrupt):
                    print()
                    break
                if not q:
                    continue
                run_stream(q, args.model, tts, player,
                           num_predict=args.num_predict)
    finally:
        player.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
