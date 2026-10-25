# Third Hand — Water, within reach

A local 3D simulation for Third Hand. It opens with a seated person beside a water cup on a nearby table. After the request “Hey Arm, I’d like some water,” a wheelchair-mounted SO-101 visual model moves a lidded cup to a sipping position and returns it when the person is done. The person, cup, camera detection, and movements are simulated. It does not connect to a microphone, cloud API, Raspberry Pi, or physical motors.

## Run

```sh
cd simulation
npm ci
npm run dev
```

Open the localhost URL printed by Vite. The guided sequence plays automatically and can be replayed. **Start manually** runs the same scene with a button to confirm the detected cup and a button to return it. Space pauses or resumes motion; Escape stops motion; **Reset scene** restores the starting pose. Drag to orbit the 3D camera.

## Notes

- The wheelchair user model is [Innerscene's CC0 seated adult in a manual wheelchair](https://www.innerscene.com/tools/library/3d-parts/seated-adult-in-a-manual-wheelchair-05d08277). It is a static mesh and stays still throughout the simulation. The scene waits for it and the SO-101 model to load before showing either, so there is no temporary stand-in.
- Three.js requests a high-performance WebGL context. The top bar reports whether the browser selected a GPU or software renderer; the OS and browser make the final choice.
- The arm uses the [official SO-101 URDF and visual meshes](https://github.com/TheRobotStudio/SO-ARM100) under Apache 2.0. The upstream license is included at `public/models/so101/LICENSE`. Joint values are calculated locally for rendering only. This is not the robot's calibrated IK or a motor control pipeline.
- The local wood textures are [Poly Haven Wood Table](https://polyhaven.com/a/wood_table) and [Wood Floor](https://polyhaven.com/a/wood_floor), both CC0.
- The staged route checks tool-point reach and a few coarse clearances; it does not solve full link collision, force, or cup stability. `joint_settings.json` is shown only as a saved soft-bound reference. No servo ticks are generated.
- No validated filled-cup payload rating appears in the repository. The lightweight cup grasp, level liquid, camera detection, and user comfort position remain illustrative. A real system would require calibrated geometry, collision checking, force limits, payload testing, reliable perception, and supervised trials before use near a person.
- The simulation is an independent browser app. It does not send commands to the Pi.
