"""Credential-free contract tests for live Cartesia transcription."""

import asyncio
import sys
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from streaming_stt import StreamingSTT


class FakeSocket:
    def __init__(self, *, fail=False):
        self.fail = fail
        self.events = asyncio.Queue()
        self.chunks = []
        self.commands = []
        self.closed = False
        self._fed = False

    async def __aenter__(self):
        if self.fail:
            raise ConnectionError("fake disconnect")
        return self

    async def __aexit__(self, *_):
        self.closed = True

    async def send_raw(self, data):
        self.chunks.append((time.monotonic(), data))
        if not self._fed and any(data):
            self._fed = True
            for kind, transcript in (
                ("turn.start", None),
                ("turn.update", "open"),
                ("turn.update", "open gripper"),
                ("turn.eager_end", "open gripper"),
                ("turn.resume", None),
                ("turn.end", "open gripper"),
                ("turn.end", "open gripper"),
            ):
                await self.events.put(SimpleNamespace(type=kind, transcript=transcript))

    async def send(self, command):
        self.commands.append(command)
        await self.events.put(None)

    def __aiter__(self):
        return self

    async def __anext__(self):
        event = await self.events.get()
        if event is None:
            raise StopAsyncIteration
        return event


class FakeClient:
    def __init__(self, sockets, options):
        self.sockets = sockets
        self.options = options
        self.stt = SimpleNamespace(auto_finalize=self)

    def websocket(self, **kwargs):
        self.options.append(kwargs)
        return self.sockets.pop(0)


class StreamingSTTTests(unittest.TestCase):
    def make_stt(self, sockets, turns, errors, signal):
        options = []

        def on_turn(text, timing):
            turns.append((text, timing))
            signal.set()

        stt = StreamingSTT(
            "fake-key", on_turn, errors.append,
            client_factory=lambda **_: FakeClient(sockets, options),
        )
        return stt, options

    def test_pcm_pacing_turn_end_only_and_close(self):
        socket = FakeSocket()
        turns, errors = [], []
        signal = threading.Event()
        stt, options = self.make_stt([socket], turns, errors, signal)
        # Five 20 ms frames aggregate to one 100 ms PCM chunk.
        frame = b"\x01\x00" * 320
        for _ in range(5):
            stt.send(frame)
        stt.start()
        self.assertTrue(signal.wait(2))
        time.sleep(0.22)
        stt.close()
        self.assertEqual(len(turns), 1)
        text, timing = turns[0]
        self.assertEqual(text, "open gripper")
        self.assertGreaterEqual(timing["turn_ms"], 0)
        self.assertGreaterEqual(timing["stt_ms"], 0)
        self.assertGreaterEqual(timing["first_partial_ms"], 0)
        self.assertEqual(timing["finalize_ms"], 0)  # eager end was resumed
        self.assertEqual(timing["stt_ms"], timing["finalize_ms"])
        self.assertEqual(options[0], {
            "model": "ink-2", "encoding": "pcm_s16le", "sample_rate": 16000,
            "turn_end_timeout_ms": 640,
        })
        self.assertEqual(socket.chunks[0][1], frame * 5)
        self.assertEqual(socket.chunks[1][1], b"\0" * 3200)
        self.assertGreaterEqual(socket.chunks[1][0] - socket.chunks[0][0], 0.075)
        self.assertEqual(socket.commands, [{"type": "close"}])
        self.assertTrue(socket.closed)
        self.assertEqual(errors, [])

    def test_backlog_drops_oldest_audio(self):
        stt, _ = self.make_stt([], [], [], threading.Event())
        stale = b"\x03\x00" * 1600
        old = b"\x01\x00" * 1600
        new = b"\x02\x00" * 1600
        stt.send(stale)
        for _ in range(19):
            stt.send(old)
        stt.send(new)
        self.assertEqual(len(stt._audio), stt._max_bytes)
        self.assertEqual(stt._audio[-len(new):], new)
        self.assertEqual(stt._audio[:len(old)], old)
        stt.close()

    def test_transient_disconnect_recovers_without_fatal_callback(self):
        failed, recovered = FakeSocket(fail=True), FakeSocket()
        turns, errors = [], []
        signal = threading.Event()
        stt, options = self.make_stt([failed, recovered], turns, errors, signal)
        stt.send(b"\x01\x00" * 1600)
        stt.start()
        self.assertTrue(signal.wait(2))
        stt.close()
        self.assertEqual(turns[0][0], "open gripper")
        self.assertEqual(errors, [])
        self.assertEqual(len(options), 2)
        self.assertTrue(recovered.closed)

    def test_permanent_failure_reports_once_and_stops(self):
        errors = []
        terminal = threading.Event()

        def on_error(error):
            errors.append(error)
            terminal.set()

        sockets = [FakeSocket(fail=True) for _ in range(3)]
        stt = StreamingSTT(
            "fake-key", lambda *_: None, on_error,
            client_factory=lambda **_: FakeClient(sockets, []),
        )
        stt.start()
        self.assertTrue(terminal.wait(2))
        self.assertEqual(len(errors), 1)
        self.assertIsInstance(errors[0], RuntimeError)
        self.assertNotIn("fake disconnect", str(errors[0]))
        self.assertTrue(stt._closing.is_set())
        stt.close()


if __name__ == "__main__":
    unittest.main()
