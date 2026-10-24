# Centering the jaws over a named object

`python -m care_reach.routines.center "coke can"` finds the object in the wrist
camera's picture and nudges the arm until the jaws are right over it. It is the
first step of the v1 pick. Lowering, gripping, and dropping come next.

## How it works

1. The arm moves to the **survey pose**, with the gripper pointing down at the table.
2. It takes a picture and asks Grounding DINO for the object.
3. The **aim point** is where an object appears in the picture when the jaws are
   right over it. If the object isn't there, the arm nudges sideways toward it
   and looks again.
4. Pixels convert to meters with an estimate from the lens spec (102° across
   1280 px) and the survey height. Each nudge covers 80% of the estimated
   distance so it doesn't overshoot, and the next look corrects the rest.
5. After four looks without lining up, it stops and reports "could not center".
   If the object isn't in the picture, it reports "not found".

Nudges follow the gripper's own x and y axes and stay level. The camera turns
with the gripper, so the same picture direction always means the same axis.

## What runs where

| Piece | Where | Notes |
| --- | --- | --- |
| Camera service | Pi, `care-reach-camera.service` | ustreamer owns the camera and serves `http://127.0.0.1:8766/snapshot`. It is set up on the Pi, not in this repo. |
| Detector | GPU pod, `care_reach.inference.detect_server` | Grounding DINO base on `127.0.0.1:8050`, token-protected. |
| Tunnel | Pi to pod | `ssh -f -N -i ~/.ssh/care_reach_detector_ed25519 -p <pod ssh port> -L 8050:127.0.0.1:8050 root@<pod ip>`. The pod accepts that key only for forwarding to port 8050. |
| `pick_setup`, `center` | Pi | Need `CARE_REACH_DETECTOR_URL=http://127.0.0.1:8050` and `CARE_REACH_DETECTOR_TOKEN`. |

Start the detector on the pod from a copy of this package:

```bash
CARE_REACH_DETECTOR_TOKEN=<secret> /root/lerobot/.venv/bin/python -m care_reach.inference.detect_server
```

## One-time setup

1. **Aim.** With the dashboard running, jog the arm so the gripper points down
   about 20 cm above the table, with a short test object right under the jaws.
   Measure the camera lens's height above the table, then run
   `python -m care_reach.routines.pick_setup aim "coke can" --survey-height-cm 20`.
   It saves the pose, height, and aim point to `config/pick.json`.
2. **Direction.** Leave the object in place, stop the dashboard, and run
   `python -m care_reach.routines.pick_setup direction "coke can"`. The arm
   nudges 2 cm along each gripper axis and saves which picture direction is
   which axis. It refuses a camera mounted crooked, or a picture scale outside
   0.8–1.7 times the lens estimate. Outside that range, four looks can't
   center an object 10 cm away, so check the survey height. Rerun it after
   every aim.

## Running it

Without `--execute`, `center` takes one picture from wherever the arm is and
prints where the object is relative to the jaws. With `--execute`, stop the
dashboard first: the arm moves to the survey pose and centers. Each look is
logged with the object's pixel position and its distance from the jaws.

## Live findings, 2026-10-24

On the Pi, the camera service returned a fresh 1280×720 picture in about
20 ms. Grounding DINO on the pod answered in 150–270 ms. Real prompts found a
laptop (0.52), a hand (0.75), a blue cable (0.68), and a couch armrest (0.67).

- **The gripper's own jaws show at the bottom of the picture.** For objects
  that weren't there, such as "coke can" and "water bottle", the jaws scored up
  to 0.37, so the detector ignores matches below 0.45.
- **The Pi's Wi-Fi stalls.** Uploading one 150 KB picture took 13 s once and
  0.2 s the next time. The detector client waits up to 10 s, while the arm
  holds still.
