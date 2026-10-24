# Raspberry Pi voice test

The Pi uses its USB PnP microphone through FFmpeg ALSA (`hw:2,0`). Cartesia Ink-Whisper transcribes speech; OpenRouter GPT-4.1 Mini selects a command from a fixed allowlist. The dashboard shows mic level, exact transcript, decision, action result, and API times. Labeled trials generate a missed trigger, wrong command, and unwanted activation report.

Exact commands such as **“Hey Arm, open the gripper”** and **“Hey Arm, close the gripper”** use local phrase matching after Cartesia transcription for a faster response. Other wording goes to OpenRouter. The Pi closes a speech segment after 600 ms of silence; the dashboard's response time starts at the end of that segment.

If OpenRouter rejects its key or is unavailable, the page switches to strict local phrase matching and shows that under **Command path**. Replace the key in the protected Pi environment file and restart the service to retry OpenRouter.

The Pi voice page runs on `127.0.0.1:8770`. From the Mac, use an SSH tunnel:

```sh
ssh -N -L 127.0.0.1:8770:127.0.0.1:8770 care-reach-pi
```

Open `http://127.0.0.1:8770/` in a browser. The user service is `care-reach-pi-voice.service`.

Speech decisions start in dry-run mode. To test physical motion, keep fingers clear of the gripper, press **Enable one gripper move**, then say **“Hey Arm, open the gripper”** or **“Hey Arm, close the gripper”** within 120 seconds. Each command moves the gripper by at most 120 encoder ticks, within the dashboard's measured bounds. It disables motion after that one action. **“Hey Arm, stop”** calls the existing dashboard stop endpoints. **“Hey Arm, pick up the can”** is recognized and shown as unsupported; camera localization and grasp planning are not connected.

Credentials are read from `/home/robot/.config/care-reach/voice.env` on the Pi. This file must be mode 600 and never committed. Cartesia and OpenRouter are cloud services and may charge for usage.
