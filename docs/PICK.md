# Picking up a named object

`python pick_up.py "coke can" --execute` picks up any object both cameras can
see and the jaws can span: it finds the object by name, reaches it, grips
until contact, and lifts it 3 cm. Without `--execute` it only looks and plans.
Torque stays on, so the jaws keep holding the object afterward; open them from
the dashboard.

Nothing in it is specific to one kind of object. Every object is located,
modeled, and grasped the same way.

## How it works

1. **Find.** Grounding DINO on the GPU pod boxes the object in a fresh
   picture from each camera: the wrist camera on the gripper, and the
   shoulder camera (the w200) low on the base's turning part.
2. **Locate.** Each camera's calibration turns its box into lines of sight
   in the arm's base frame. Where the two cameras' lines meet is a first
   estimate. A box's middle isn't the object's middle, though (a camera
   looking down sees the top as well as the side), so the estimate is refined
   to the upright cylinder whose outline fills both boxes. That gives the
   object's position, width, and top and bottom heights; the bottom is the
   surface it rests on, whatever its height.
3. **Plan.** The jaws close horizontally across the object, 2 cm below its
   top (or at its middle if it is short), with the fixed jaw 5 mm outside its
   near side. A five-joint SO-101 can only point its jaws within the upright
   plane through the base's pan axis, so the planner tries tilts from straight
   down to level and keeps the first whose 6 cm straight approach the joints
   reach without any part of the gripper coming within 1 cm of the object or
   its surface. Each joint move is checked the same way, in the steps the
   measured mover takes.
4. **Look again, up close.** The arm's kinematic model is off by a couple of
   centimeters across its reach, but the error barely changes over a short
   move. The arm moves near the object, posed so the wrist camera (which sits
   behind and beside the jaws and looks past them) sees all of it from 15–25
   cm, and locates it again. The plan from there is where the jaws will go.
5. **Approach, grip, lift.** The open jaws move to the pre-grasp and straight
   in, 1 cm at a time; the measured mover stops the pick if anything stalls
   them. The jaws close until they stop on something and keep a 30-tick
   squeeze, the dashboard's. A miss reopens them and backs out. The jaws rise
   3 cm, and the pick checks they still hold something.

| Module | Role |
| --- | --- |
| `pick/camera.py` | Fresh pictures from the Pi's camera services. |
| `pick/rig.py` | Camera calibrations, and lines of sight in the base frame. |
| `pick/calibration.py` | Solving each camera's lens and mount from ChArUco pictures. |
| `pick/detector.py` | Grounding DINO client; ignores weak or ambiguous matches. |
| `pick/locate.py` | Where the object is and how big, from both boxes. |
| `pick/grasp.py` | The close look and the collision-checked approach. |
| `pick/arm.py` | Joint moves, straight lifts, and the gripper, on the measured mover. |
| `pick/routine.py` | The whole pick. |
| `pick/detect_server.py` | The detector's HTTP server, run on the GPU pod. |

## What runs where

| Piece | Where | Notes |
| --- | --- | --- |
| Camera services | Pi | ustreamer serves the wrist camera on port 8766 and the shoulder camera on 8768; see [the camera notes](CAMERA.md). Pictures come from the MJPEG streams, which are fresh; `/snapshot` can be seconds old. |
| Detector | GPU pod | `CARE_REACH_DETECTOR_TOKEN=<secret> python -m pick.detect_server` from a copy of `pick/`, with torch, transformers, and Pillow. It listens on pod localhost port 8050. |
| Tunnel | Pi to pod | `ssh -f -N -i ~/.ssh/care_reach_detector_ed25519 -p <pod ssh port> -L 8050:127.0.0.1:8050 root@<pod ip>`. The pod accepts that key only for forwarding to port 8050. |
| `calibrate_cameras.py`, `pick_up.py` | Pi | Stop the dashboard first. `pick_up.py` needs `CARE_REACH_DETECTOR_URL=http://127.0.0.1:8050` and `CARE_REACH_DETECTOR_TOKEN`, plus the usual `PYTHONPATH` for the motor-only LeRobot. |

## Calibrating the cameras

Once, and again whenever a camera is knocked on its mount:

1. `python calibrate_cameras.py pattern board.png` writes the 9 × 6-square
   ChArUco board. Show it full screen at one image pixel per screen pixel
   (its squares are then one inch on a 224 ppi MacBook Air), or print it, and
   pass its measured square size with `--square-mm` if it isn't 25.4 mm.
2. **Wrist.** Set the board down in front of the arm and pose the arm from
   the dashboard so the wrist camera sees it. Stop the dashboard and run
   `python calibrate_cameras.py wrist`. The arm visits poses around
   that one, never lower or farther out, and saves the lens and the camera's
   mount on the gripper.
3. **Shoulder.** Prop the board up at a slant, facing the shoulder camera,
   with the arm posed so the wrist camera sees it too. Run
   `python calibrate_cameras.py shoulder`. The base swings about 26 degrees
   each way, so keep that arc clear. The shoulder camera's own pictures place
   the pan axis relative to the board, and the wrist camera places the board.

Each run saves its pictures under `~/.local/state/care-reach/calibration/`
and its result in `~/.local/state/care-reach/cameras.json`;
`--from <folder>` solves saved pictures again without moving the arm.

The 2026-10-25 calibration, from a laptop screen: the wrist camera fits its
pictures to 0.83 px, and its 16 views put the still board in the same place
to 5.6 mm. The shoulder camera fits to 1.98 px through the whole chain, and
its pictures place it on the shoulder to 0.4 mm. The two cameras' views of
the same board differed by about 5 degrees and 2 cm, which is the arm
model's error between the base and the gripper; the close look is there
because of it.

## Limits

- The object must be fully inside both pictures from where the arm starts;
  a box at a picture's edge stops the pick rather than guessing.
- Objects are modeled as upright cylinders: a box or bottle fits well, an
  object lying on its side less so. Up to 8 cm wide.
- Nothing watches the object during the final approach. If it moves, the
  jaws close on nothing and the pick reports "missed".
- Contact is detected when the gripper stops moving. A very thin or soft
  object may read as a miss.
