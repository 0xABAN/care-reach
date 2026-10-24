# Cartesia voice test on Mac

This test listens to the USB microphone through FFmpeg, detects utterances locally, and sends each detected utterance to Cartesia's hosted `ink-whisper` speech-to-text model. It shows the transcript, “Hey Arm” wake decision, selected command, and API response time at `http://127.0.0.1:8765`. The dashboard only binds to this Mac. No Raspberry Pi or arm motors are connected.

## Start

```sh
/Users/ved/Documents/ChatGPT/calhacks/voice-test/run.sh
```

The supplied Cartesia key is stored in this Mac's Keychain under service `calhacks-cartesia` and account `calhacks-voice`. The key is not stored in source files. On another computer, set `CARTESIA_API_KEY` before running. The project uses a local Python environment in `.venv`; install the official client with `uv pip install --python voice-test/.venv/bin/python 'cartesia[websockets]'` if rebuilding that environment.

The connected USB mic appeared as FFmpeg AVFoundation audio device `1`. If devices change, list them with `ffmpeg -hide_banner -f avfoundation -list_devices true -i ''` and pass `--device N` to `run.sh`.

Say “Hey Arm, pick up the can” in one phrase, or say “Hey Arm” and then the command within twelve seconds. Cartesia has also transcribed this user's “Hey Arm” as “Hey, um,” so the prototype accepts that exact variant as a wake candidate. Supported commands are can pickup, `stop`, and `cancel`. All commands require a wake candidate. For scored trials, use the dashboard buttons before speaking. The report at `reports/latest.md` records missed triggers, wrong commands, and unwanted activations from labeled trials. It does not prove safe operation.

The mic stays on while the program runs, but only utterances detected by the local voice activity threshold are uploaded. Use **Pause listening** in the dashboard to stop uploads when finished. Cartesia API usage may be billed. The temporary upload WAV is deleted after each request. Text results are stored in `reports/events-cartesia-ink-whisper.csv`. **Save next utterance** deliberately keeps one local WAV in `reports/last-utterance.wav` for replay.
