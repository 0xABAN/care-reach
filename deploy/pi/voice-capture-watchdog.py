#!/usr/bin/env python3
"""Restart the Pi voice service if its USB capture silently stops."""
import glob
import json
import subprocess
import time
from urllib.request import Request, urlopen

SERVICE = "care-reach-pi-voice.service"
URL = "http://127.0.0.1:8770"
DEVICE_LABEL = "USB PnP Sound Device"


def state():
    try:
        with urlopen(URL + "/state", timeout=2) as response:
            return json.load(response)
    except Exception:
        return {}


def capture_running():
    for path in glob.glob("/proc/asound/card*/stream0"):
        try:
            stream = open(path, encoding="utf-8").read()
        except OSError:
            continue
        if DEVICE_LABEL in stream:
            return "Capture:\n  Status: Running" in stream
    return False


def main():
    active = subprocess.run(["systemctl", "--user", "is-active", "--quiet", SERVICE],
                            check=False).returncode == 0
    if active and capture_running():
        print("voice capture running")
        return
    was_listening = bool(state().get("listening")) if active else False
    reason = "USB capture stopped" if active else "voice service inactive"
    print(f"{reason}; restarting {SERVICE}", flush=True)
    subprocess.run(["systemctl", "--user", "restart" if active else "start", SERVICE],
                   check=True)
    for _ in range(24):
        if state() and capture_running():
            break
        time.sleep(0.25)
    else:
        raise RuntimeError("voice capture did not recover")
    if was_listening:
        request = Request(URL + "/listen", data=b'{"enabled":true}',
                          headers={"Content-Type": "application/json"})
        with urlopen(request, timeout=5) as response:
            if response.status != 200:
                raise RuntimeError("could not restore listening")
        print("voice capture recovered; listening restored")
    else:
        print("voice capture recovered; listening remains paused")


if __name__ == "__main__":
    main()
