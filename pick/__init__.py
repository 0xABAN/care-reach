"""Picking up a named object with the wrist camera: look, center over it, grab.

- ``camera``: pictures from the Pi's camera service.
- ``detector``: Grounding DINO on the GPU pod finds the object by name.
- ``config``: what `pick_setup.py` recorded about the survey pose and camera.
- ``arm``: poses, level nudges, vertical moves, and the gripper.
- ``center``: look and nudge until the jaws are over the object.
- ``routine``: the whole pick, from survey pose to lifted object.
- ``detect_server``: the detector's HTTP server, run on the GPU pod.
"""
