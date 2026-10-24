# Picking up a named object

`python pick_up.py "blue box" --execute` finds the object in the wrist
camera's picture, centers the jaws over it, lowers them around it, grips, and
lifts it back to the survey height. Torque stays on, so the jaws keep holding
the object afterward; open them from the dashboard.

## How it works

1. **Survey.** The arm moves to the survey pose, with the gripper pointing
   straight down at the table.
2. **Center.** It takes a picture and asks Grounding DINO for the object. The
   **aim point** is where an object appears in the picture when the jaws are
   right over it. If the object isn't there, the arm nudges sideways toward it
   and looks again, like parallel parking. Pixels convert to meters with an
   estimate from the lens spec (102° across 1280 px) and the lens height; each
   nudge covers 80% of the estimate so it doesn't overshoot. After four looks
   it reports "could not center"; if the object isn't in the picture, "not
   found".
3. **Grab.** The jaws open, go straight down until their tips are 2 cm above
   the table (`--grasp-height-cm`), and close until they stop on something.
   The gripper then keeps a goal 30 ticks inside the contact point, the same
   squeeze the dashboard keeps after a closing jog.
4. **Lift.** The arm rises back to the survey height while the jaws keep
   squeezing. If the jaws closed on nothing, they reopen first and it reports
   "missed". If the object slips out on the way up, it reports "dropped".

Nudges follow the gripper's own x and y axes and stay level; vertical moves go
straight up or down in the arm's base frame. Both run as short IK legs through
`so101_ik.py`, each planned from the pose the encoders measured after the last
one, and each leg runs on `MeasuredPoseRunner` with its usual checks.

| Module | Role |
| --- | --- |
| `pick/camera.py` | Pictures from the Pi's camera service. |
| `pick/detector.py` | Grounding DINO client; ignores weak or ambiguous matches. |
| `pick/config.py` | The survey pose, heights, aim point, and camera directions. |
| `pick/arm.py` | Poses, nudges, vertical moves, and the gripper, on the measured mover. |
| `pick/center.py` | Look and nudge until the jaws are over the object. |
| `pick/routine.py` | The whole pick. |
| `pick/detect_server.py` | The detector's HTTP server, run on the GPU pod. |

`MeasuredPoseRunner.close_gripper` closes until contact and leaves the squeeze
in place. `MeasuredPoseRunner.move(..., holding_object=True)` moves the arm
while keeping that squeeze, leaving the blocked, loaded gripper out of the
arrival, progress, and load checks.

## What runs where

| Piece | Where | Notes |
| --- | --- | --- |
| Camera service | Pi, `care-reach-camera.service` | ustreamer owns the camera; see [the camera notes](CAMERA.md). The camera must be on the wrist mount, looking along the jaws. |
| Detector | GPU pod | `CARE_REACH_DETECTOR_TOKEN=<secret> python -m pick.detect_server` from a copy of this repo, using the pod's LeRobot environment. It listens on pod localhost port 8050. |
| Tunnel | Pi to pod | `ssh -f -N -i ~/.ssh/care_reach_detector_ed25519 -p <pod ssh port> -L 8050:127.0.0.1:8050 root@<pod ip>`. The pod accepts that key only for forwarding to port 8050. |
| `pick_setup.py`, `pick_up.py` | Pi | Need `CARE_REACH_DETECTOR_URL=http://127.0.0.1:8050` and `CARE_REACH_DETECTOR_TOKEN`, plus the usual `PYTHONPATH` for the motor-only LeRobot. |

## One-time setup

1. **Aim.** With the dashboard running, jog the arm so the gripper points
   straight down about 20 cm above the table, with a short test object right
   under the jaws. Measure the camera lens's and the jaw tips' heights above
   the table, then run
   `python pick_setup.py aim "blue box" --lens-height-cm 22 --jaw-height-cm 14`.
   It saves the pose, heights, and aim point to `pick_config.json`.
2. **Direction.** Leave the object in place, stop the dashboard, and run
   `python pick_setup.py direction "blue box"`. The arm nudges 2 cm along each
   gripper axis and saves which picture direction is which axis. It refuses a
   camera mounted crooked, or a picture scale outside 0.8–1.7 times the lens
   estimate: outside that range, four looks can't center an object 10 cm away,
   so check the lens height. Rerun it after every aim.

## Running it

Without `--execute`, `pick_up.py` takes one picture from wherever the arm is
and prints where the object is relative to the jaws; nothing moves. With
`--execute`, stop the dashboard first; the command refuses to start while
another program owns the URT-2. `--center-only` stops once the jaws are over
the object, a good first live test. Each look is logged with the object's pixel
position and its distance from the jaws.

Any safety stop holds the measured pose with torque on and prints `stopped:`
with the reason.

## Limits

- Short objects only. The aim point is recorded for a short object; a tall one
  appears shifted from the camera's viewpoint, and the gripper body would hit
  its top before the jaw tips reach 2 cm.
- The object must be visible from the survey pose.
- Nothing watches the object while the jaws go down. If it moves, they close
  on nothing and the pick reports "missed".
- The kinematic model's heights haven't been checked against a ruler (see
  [kinematics](KINEMATICS.md)). Start with a larger `--grasp-height-cm`, such
  as 4, and lower it once the jaws stop where expected.
- Contact is detected when the gripper stops moving. A very thin or soft
  object may read as a miss.

## Live findings, 2026-10-24

On the Pi, the camera service returned a fresh 1280×720 picture in about
20 ms. Grounding DINO on the pod answered in 150–270 ms. Real prompts found a
laptop (0.52), a hand (0.75), a blue box (0.70), and a clear plastic box
(0.89). "box" alone matched the plastic box, so name the object's color or
material when two similar objects are in view.

- **The gripper's own jaws show at the bottom of the picture.** For objects
  that weren't there, such as "coke can" and "water bottle", the jaws scored up
  to 0.37, so the detector ignores matches below 0.45.
- **The Pi's Wi-Fi stalls.** Uploading one 150 KB picture took 13 s once and
  0.2 s the next time. The detector client waits up to 10 s, while the arm
  holds still.
