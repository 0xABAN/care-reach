"""Picking up a named object that both cameras can see.

- ``camera``: fresh pictures from the Pi's camera services.
- ``rig``: the calibrated cameras, and lines of sight in the arm's base frame.
- ``calibration``: solving each camera's lens and mount from ChArUco board pictures.
- ``detector``: Grounding DINO on the GPU pod finds the object by name.
- ``locate``: where the object is, from both cameras' lines of sight.
- ``grasp``: a reachable, collision-free approach for the open jaws.
- ``arm``: joint moves, straight lifts, and the gripper, on the measured mover.
- ``routine``: the whole pick, from first look to lifted object.
- ``detect_server``: the detector's HTTP server, run on the GPU pod.
"""
