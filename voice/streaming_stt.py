"""Continuous, paced Cartesia Ink 2 transcription for a PCM microphone stream."""

from __future__ import annotations

import asyncio
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Callable


class StreamingSTT:
    """Stream 16-bit mono PCM and report only confirmed, complete turns.

    ``send`` is safe to call from a microphone thread. Its two-second buffer
    favors current audio if the network falls behind. Short input gaps are sent
    as zero samples so Cartesia can distinguish silence from a stalled stream.
    ``stt_ms`` aliases ``finalize_ms``: the interval from an eager end to the
    confirmed end. Both are zero if no eager end remains after a resume.
    """

    def __init__(self, api_key: str, on_turn_end: Callable, on_error: Callable,
                 *, model: str = "ink-2", sample_rate: int = 16000,
                 turn_end_timeout_ms: int = 640, client_factory=None):
        if sample_rate <= 0 or sample_rate % 10:
            raise ValueError("sample_rate must be a positive multiple of 10")
        if not 640 <= turn_end_timeout_ms <= 11200:
            raise ValueError("turn_end_timeout_ms must be between 640 and 11200")
        self.api_key = api_key
        self.on_turn_end = on_turn_end
        self.on_error = on_error
        self.model = model
        self.sample_rate = sample_rate
        self.turn_end_timeout_ms = turn_end_timeout_ms
        self._client_factory = client_factory
        self.chunk_bytes = sample_rate * 2 // 10
        self._max_bytes = self.chunk_bytes * 20
        self._audio = bytearray()
        self._audio_lock = threading.Lock()
        self._closing = threading.Event()
        self._thread: threading.Thread | None = None
        self._callbacks: ThreadPoolExecutor | None = None

    def start(self) -> None:
        if self._thread is not None:
            return
        if self._closing.is_set():
            raise RuntimeError("StreamingSTT is closed")
        if self._client_factory is None:
            from cartesia import AsyncCartesia
            self._client_factory = AsyncCartesia
        self._callbacks = ThreadPoolExecutor(max_workers=1, thread_name_prefix="stt-callback")
        self._thread = threading.Thread(target=self._run, name="cartesia-stt", daemon=True)
        self._thread.start()

    def send(self, frame: bytes) -> None:
        if self._closing.is_set() or not frame:
            return
        if len(frame) % 2:
            raise ValueError("PCM s16le frame must contain whole samples")
        with self._audio_lock:
            self._audio.extend(frame)
            excess = len(self._audio) - self._max_bytes
            if excess > 0:
                del self._audio[:excess]

    def close(self) -> None:
        self._closing.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=4)
        if self._callbacks is not None:
            self._callbacks.shutdown(wait=False, cancel_futures=True)

    def _next_chunk(self) -> bytes:
        with self._audio_lock:
            chunk = bytes(self._audio[:self.chunk_bytes])
            del self._audio[:len(chunk)]
        return chunk.ljust(self.chunk_bytes, b"\0")

    def _notify(self, callback: Callable, *args) -> None:
        if self._callbacks is not None:
            try:
                self._callbacks.submit(callback, *args)
            except RuntimeError:
                pass  # close() has already shut down the callback worker

    def _run(self) -> None:
        try:
            asyncio.run(self._main())
        except Exception as error:
            self._notify(self.on_error, RuntimeError(
                f"Cartesia STT stopped: {type(error).__name__}"))

    async def _main(self) -> None:
        retry_seconds = 0.25
        failures = 0
        while not self._closing.is_set():
            connected_at = time.monotonic()
            try:
                client = self._client_factory(api_key=self.api_key)
                async with client.stt.auto_finalize.websocket(
                    model=self.model, encoding="pcm_s16le",
                    sample_rate=self.sample_rate,
                    turn_end_timeout_ms=self.turn_end_timeout_ms,
                ) as ws:
                    await self._session(ws)
                retry_seconds = 0.25
            except Exception as error:
                if not self._closing.is_set():
                    if time.monotonic() - connected_at >= 10:
                        failures = 0
                        retry_seconds = 0.25
                    failures += 1
                    if failures >= 3:
                        self._closing.set()
                        self._notify(self.on_error, RuntimeError(
                            f"Cartesia STT unavailable after {failures} attempts "
                            f"({type(error).__name__})"))
                        return
                    await asyncio.sleep(retry_seconds)
                    retry_seconds = min(retry_seconds * 2, 2.0)

    async def _session(self, ws) -> None:
        turn_started: float | None = None
        first_partial: float | None = None
        eager_end: float | None = None
        last_update = ""
        ended = False

        async def sender() -> None:
            period = 0.1
            next_send = time.monotonic()
            while not self._closing.is_set():
                await ws.send_raw(self._next_chunk())
                next_send += period
                await asyncio.sleep(max(0, next_send - time.monotonic()))
                if next_send < time.monotonic() - period:
                    next_send = time.monotonic()
            await ws.send({"type": "close"})

        async def receiver() -> None:
            nonlocal turn_started, first_partial, eager_end, last_update, ended
            async for event in ws:
                kind = getattr(event, "type", None)
                if kind == "turn.start":
                    turn_started = time.monotonic()
                    first_partial = None
                    eager_end = None
                    last_update = ""
                    ended = False
                elif kind == "turn.update":
                    if ended:
                        turn_started = time.monotonic()
                        first_partial = None
                        eager_end = None
                        ended = False
                    if first_partial is None:
                        first_partial = time.monotonic()
                    last_update = getattr(event, "transcript", None) or last_update
                elif kind == "turn.eager_end":
                    eager_end = time.monotonic()
                elif kind == "turn.resume":
                    eager_end = None
                elif kind == "turn.end" and not ended:
                    now = time.monotonic()
                    transcript = getattr(event, "transcript", None) or last_update
                    if transcript:
                        finalize_ms = round((now - eager_end) * 1000) if eager_end else 0
                        timing = {
                            "turn_ms": round((now - (turn_started or now)) * 1000),
                            "first_partial_ms": round((first_partial - turn_started) * 1000)
                            if first_partial is not None and turn_started is not None else 0,
                            "finalize_ms": finalize_ms,
                            "stt_ms": finalize_ms,
                        }
                        self._notify(self.on_turn_end, transcript, timing)
                    turn_started = None
                    first_partial = None
                    eager_end = None
                    last_update = ""
                    ended = True
                elif kind == "error":
                    raise RuntimeError("Cartesia STT server error")
            if not self._closing.is_set():
                raise ConnectionError("Cartesia STT stream ended")

        sending = asyncio.create_task(sender())
        receiving = asyncio.create_task(receiver())
        try:
            done, pending = await asyncio.wait(
                (sending, receiving), return_when=asyncio.FIRST_COMPLETED)
            for task in done:
                task.result()
            if sending in done and self._closing.is_set():
                try:
                    await asyncio.wait_for(receiving, timeout=2.0)
                except asyncio.TimeoutError:
                    pass
        finally:
            for task in (sending, receiving):
                if not task.done():
                    task.cancel()
            await asyncio.gather(sending, receiving, return_exceptions=True)
