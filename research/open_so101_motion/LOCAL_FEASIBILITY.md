# Using the downloaded SO-101 data on this arm

Computed 2026-10-24 from all 67,557 action/state frames in `ddduk_pick_place_v3/data/`, this arm's `care_reach_follower.json` calibration, and `joint_settings.json`. This is offline analysis; no dataset action was sent to the arm.

## Coordinate conversion

LeRobot 0.6.2 uses the midpoint of each calibrated tick range as zero degrees for joints 1–5. For a dataset action `q` in degrees:

`raw_tick = (range_min + range_max)/2 + q × 4095/360`.

For the gripper action `g` on a 0–100 scale:

`raw_tick = 1317 + g × (2762−1317)/100 = 1317 + 14.45g`.

These are numeric conversions, **not proof that the source arm's physical zero pose matches this arm's pose**. The 2000-tick home on this arm corresponds to `[12.62°, −5.54°, 5.80°, 1.19°, −4.18°, 47.27%]` in LeRobot's local calibrated coordinates. Every converted command still needs this arm's soft-limit check and a measured-position tracking check before movement.

## Feasibility against current limits and speed

The table uses consecutive frames within the same episode at 30 Hz. A 90-tick/s test pace allows only 3 ticks per frame. `Outside` is the share of all dataset action frames beyond this arm's current saved soft limits after numeric conversion. It does not assess table clearance or coupled-joint collisions.

| Joint | Dataset 95th percentile action change (ticks/frame) | Frames faster than 90 ticks/s | Frames outside our soft limit |
| --- | ---: | ---: | ---: |
| 1 shoulder pan | 17 | 26.9% | 0% |
| 2 shoulder lift | 26 | 51.8% | 14.0% |
| 3 elbow flex | 24 | 36.4% | 13.5% |
| 4 wrist flex | 12 | 19.4% | 1.2% |
| 5 wrist roll | 12 | 13.1% | 0% under the currently unset panel limit |
| 6 gripper | 24.8 | 16.9% | 0% |

Overall, **12,479/67,557 frames (18.47%)** and **all 128 complete episodes** include at least one out-of-limit command. The source demonstrations' median initial converted pose is approximately `[1805, 895, 3023, 2652, 2023, 1324]` ticks, far from this arm's `[2000] × 6` home; 66.4% of episode starts have joint 2 outside our soft limit and 93.8% have joint 3 outside it. Wrist-roll actions reach a converted maximum of about 3477 ticks; the assembled wrist repeatedly resisted motion near 3358, and 0.42% of action frames exceed 3350.

The downloaded controller's default per-30-Hz-tick displacement limits of `[2.5°, 3.5°, 3.5°, 2.0°, 2.7°, 6.5%]` convert to about `[28, 40, 40, 23, 31, 94]` ticks per frame, equivalent to `[853, 1194, 1194, 683, 921, 2818]` ticks/s. These are based on source demonstrations, **not safe settings for our current 90-tick/s test motion**. A safe initial software cap at that observed pace is about 3 ticks/frame at 30 Hz, with slower acceleration and independent joint bounds; actual caps should be tuned from this arm's measured responses.

## What transfers

- **Planner structure:** use the [downloaded async controller](ddduk_act35_real_sim_v1/deployment/async_chunk_controller.py) for stale-plan rejection, anchoring, chunk blending, velocity/acceleration caps, and measured-position tracking guards. Convert its units and replace its numerical defaults with values measured on this arm. Keep torque enabled on error/exit and require a verified coiled rest pose before release.
- **Data-driven priors:** source action/state trajectories reveal joint ordering, direction conventions to verify, coordination patterns, gripper usage, and candidate speed/acceleration distributions. They can be retimed for offline simulation or used for training after checking physical pose alignment. The source action minus observed state can suggest tracking-lag ranges, but temporal alignment and source-arm hardware prevent treating it as a PID calibration for this arm.
- **Pretrained behavior:** the ACT checkpoint passed a local no-hardware smoke test on LeRobot 0.6.2: finite output `(1, 35, 6)`. On this Mac's MPS device, a synthetic-camera benchmark took 4.325 s to warm up and 0.054–0.059 s for each of three later chunks, fast enough for its nominal 0.8 s replan interval in isolation. It still requires top and wrist RGB cameras at 640×480 and 30 Hz, matching viewpoint/workcell, and the exact-arm calibration. Its card reports offline and simulated results, not a measured real-robot success rate here.
- **Bus rate:** a read-only 100-call benchmark of six-motor `sync_read('Present_Position')` had about 1 ms median and 95th percentile, so bus position feedback itself can support a 30 Hz loop. End-to-end camera, inference, write, and safety-loop timing still needs measurement.

## What does not transfer

- Raw tick limits, mechanical interference, the 2000-tick home, and a gravity-safe coiled rest pose.
- Torque/current/force targets or PID gains: the dataset has positions, images, and timestamps, but no motor load, voltage, current, or applied torque channels.
- A safe direct replay or plug-in rollout: the observed source path exceeds our current limits and tested speed; its initial pose differs sharply from home; the supplied rollout can disable torque on connect/disconnect; and its camera/workcell assumptions are unverified here.

Next implementation step: wrap model or replay actions in an exact-arm coordinate adapter, per-joint and coupled-pose limit guard, smooth trajectory generator, measured-position/voltage/fault watchdog, and torque-preserving stop. Validate in offline dry runs first, then at the arm's tested low speed. Investigate the wrist resistance and power supply before considering source-speed motion: [Feetech lists this STS3215 variant for 6–7.4 V](https://www.feetechrc.com/74v-19-kgcm-plastic-case-metal-tooth-magnetic-code-double-axis-ttl-series-steering-gear.html), while the current adapter is 5 V/4 A.
