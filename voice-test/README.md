# Cartesia voice test on Mac

This test listens to the USB microphone through FFmpeg, detects utterances locally, and sends each detected utterance to Cartesia's hosted `ink-whisper` speech-to-text model. It shows the transcript, “Hey Arm” wake decision, selected command, and API response time at `http://127.0.0.1:8765`. The dashboard only binds to this Mac. No Raspberry Pi or arm motors are connected.

## Live transcript stream (Ink 2 over WebSocket)

For a minimal microphone-to-text stream with no command parsing or robot connection, install the dependencies and run:

```sh
python3 -m venv voice-test/.venv
voice-test/.venv/bin/python -m pip install -r voice-test/requirements-stream.txt
voice-test/.venv/bin/python voice-test/cartesia_mic_stream.py
```

The script reads `CARTESIA_API_KEY` from the environment (on this Mac it also checks the existing `calhacks-cartesia` Keychain item), streams 16 kHz mono microphone audio to Cartesia Ink 2, and prints cumulative partial transcripts as they arrive. Each completed utterance is printed once. Choose a microphone with `--device 1` or `--device "USB microphone name"`; list devices with `voice-test/.venv/bin/python -c 'import sounddevice as sd; print(sd.query_devices())'`. To consume events in another program, use `voice-test/.venv/bin/python voice-test/cartesia_mic_stream.py --jsonl`, which writes `partial` and `final` JSON Lines to stdout. Press Ctrl+C to stop. This sends microphone audio to Cartesia while running and may incur API charges.

On Raspberry Pi OS, install PortAudio first (`sudo apt install libportaudio2`), install the same Python requirements, and set `CARTESIA_API_KEY` in the environment before running. The listener only prints text; it does not interpret commands or control the robot.

## Start

```sh
/Users/ved/Documents/ChatGPT/calhacks/voice-test/run.sh
```

The supplied Cartesia key is stored in this Mac's Keychain under service `calhacks-cartesia` and account `calhacks-voice`. The key is not stored in source files. On another computer, set `CARTESIA_API_KEY` before running. The project uses a local Python environment in `.venv`; install the official client with `uv pip install --python voice-test/.venv/bin/python 'cartesia[websockets]'` if rebuilding that environment.

The connected USB mic appeared as FFmpeg AVFoundation audio device `1`. If devices change, list them with `ffmpeg -hide_banner -f avfoundation -list_devices true -i ''` and pass `--device N` to `run.sh`.

Say “Hey Arm, pick up the can” in one phrase, or say “Hey Arm” and then the command within twelve seconds. Cartesia has also transcribed this user's “Hey Arm” as “Hey, um,” so the prototype accepts that exact variant as a wake candidate. Supported commands are can pickup, `stop`, and `cancel`. All commands require a wake candidate. For scored trials, use the dashboard buttons before speaking. The report at `reports/latest.md` records missed triggers, wrong commands, and unwanted activations from labeled trials. It does not prove safe operation.

The mic stays on while the program runs, but only utterances detected by the local voice activity threshold are uploaded. Use **Pause listening** in the dashboard to stop uploads when finished. Cartesia API usage may be billed. The temporary upload WAV is deleted after each request. Text results are stored in `reports/events-cartesia-ink-whisper.csv`. **Save next utterance** deliberately keeps one local WAV in `reports/last-utterance.wav` for replay.
