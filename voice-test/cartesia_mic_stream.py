#!/usr/bin/env python3
"""Stream microphone audio to Cartesia Ink 2 and print transcript updates."""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import queue
import subprocess
import sys
from typing import Any


SAMPLE_RATE = 16_000
BLOCK_FRAMES = SAMPLE_RATE // 10  # 100 ms chunks
QUEUE_BLOCKS = 50  # At most five seconds of buffered microphone audio.


def get_api_key() -> str:
    key = os.environ.get("CARTESIA_API_KEY", "").strip()
    if not key and sys.platform == "darwin":
        result = subprocess.run(
            ["security", "find-generic-password", "-a", "calhacks-voice",
             "-s", "calhacks-cartesia", "-w"],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode == 0:
            key = result.stdout.strip()
    return key


def parse_device(value: str | None) -> int | str | None:
    if value is None:
        return None
    try:
        return int(value)
    except ValueError:
        return value


async def run(device: int | str | None, jsonl: bool) -> None:
    try:
        import sounddevice as sd
        from cartesia import AsyncCartesia
    except ImportError as exc:
        raise SystemExit(
            'Missing dependencies. Install with: python -m pip install '
            '"cartesia[websockets]" sounddevice'
        ) from exc

    api_key = get_api_key()
    if not api_key:
        raise SystemExit("Set CARTESIA_API_KEY (the Mac Keychain key is also checked on macOS).")

    loop = asyncio.get_running_loop()
    audio: asyncio.Queue[bytes] = asyncio.Queue(maxsize=QUEUE_BLOCKS)
    dropped_blocks = 0

    def enqueue(block: bytes) -> None:
        nonlocal dropped_blocks
        if audio.full():
            try:
                audio.get_nowait()
                dropped_blocks += 1
            except asyncio.QueueEmpty:
                pass
        audio.put_nowait(block)

    def on_audio(indata: Any, frames: int, time_info: Any, status: Any) -> None:
        if status:
            print(f"[mic] {status}", file=sys.stderr, flush=True)
        loop.call_soon_threadsafe(enqueue, bytes(indata))

    client = AsyncCartesia(api_key=api_key)
    print("[mic] Listening. Press Ctrl+C to stop.", file=sys.stderr, flush=True)
    try:
        async with client.stt.auto_finalize.websocket(
            model="ink-2",
            encoding="pcm_s16le",
            sample_rate=SAMPLE_RATE,
        ) as ws:
            async def send_audio() -> None:
                with sd.RawInputStream(
                    device=device,
                    samplerate=SAMPLE_RATE,
                    blocksize=BLOCK_FRAMES,
                    channels=1,
                    dtype="int16",
                    callback=on_audio,
                ):
                    while True:
                        await ws.send_raw(await audio.get())

            async def receive_transcripts() -> None:
                turn_text = ""
                async for event in ws:
                    event_type = event.type
                    transcript = getattr(event, "transcript", None) or ""
                    if event_type == "turn.start":
                        turn_text = ""
                        if not jsonl:
                            print("\n[listening]", file=sys.stderr, flush=True)
                    elif event_type == "turn.update":
                        turn_text = transcript or turn_text
                        emit(jsonl, "partial", turn_text)
                    elif event_type == "turn.eager_end":
                        turn_text = transcript or turn_text
                        if not jsonl:
                            print("\n[probably done]", file=sys.stderr, flush=True)
                    elif event_type == "turn.resume":
                        if not jsonl:
                            print("[still listening]", file=sys.stderr, flush=True)
                    elif event_type == "turn.end":
                        turn_text = transcript or turn_text
                        emit(jsonl, "final", turn_text)
                        turn_text = ""
                    elif event_type == "error":
                        raise RuntimeError(getattr(event, "message", "Cartesia STT error"))

            sender = asyncio.create_task(send_audio())
            receiver = asyncio.create_task(receive_transcripts())
            done, pending = await asyncio.wait(
                {sender, receiver}, return_when=asyncio.FIRST_COMPLETED
            )
            for task in pending:
                task.cancel()
            await asyncio.gather(*pending, return_exceptions=True)
            for task in done:
                error = task.exception()
                if error:
                    raise error
    finally:
        await client.close()
        if dropped_blocks:
            print(f"[mic] Dropped {dropped_blocks} audio chunks because the network fell behind.",
                  file=sys.stderr, flush=True)


def emit(jsonl: bool, event_type: str, transcript: str) -> None:
    if jsonl:
        print(json.dumps({"type": event_type, "text": transcript}, ensure_ascii=False), flush=True)
    elif event_type == "partial":
        print(f"\r{text_for_terminal(transcript)}", end="", flush=True)
    else:
        print(f"\r{text_for_terminal(transcript)}\n", flush=True)


def text_for_terminal(text: str) -> str:
    return "... " + text.replace("\n", " ")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", help="sounddevice input device index or name; defaults to system default")
    parser.add_argument("--jsonl", action="store_true", help="write partial/final transcript events as JSON Lines")
    args = parser.parse_args()
    try:
        asyncio.run(run(parse_device(args.device), args.jsonl))
    except KeyboardInterrupt:
        print("\n[stopped]", file=sys.stderr, flush=True)


if __name__ == "__main__":
    main()
