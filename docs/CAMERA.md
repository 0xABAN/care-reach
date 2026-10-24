# Pi camera live feed

The connected USB camera is `Innomaker-U20CAM-720P` (`0c45:6367`). It provides
MJPEG at 1280×720 and 30 fps. Use the stable capture device path
`/dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-720P_SN0001-video-index0`.
The Pi's user service runs `ustreamer` on **Pi localhost only**, port 8766.
The dashboard relays it under `/camera/stream` and `/camera/snapshot` on port
8765; it does not own or change the robot's serial bus.

On the Pi, install the Debian package and user service:

```bash
sudo apt-get install -y ustreamer
bash deploy/pi/install-camera.sh
systemctl --user status care-reach-camera.service
```

Open the **Camera** tab in the [regular dashboard](http://pi.local:8765/)
from a computer that can reach the Pi. The same tab works through the
[dashboard's Tailscale SSH tunnel](http://127.0.0.1:8765/) on this Mac. The
camera is available to everyone with dashboard access.

For a separate camera-only view on a Mac that can SSH to the Pi over Tailscale:

```bash
bash deploy/mac/camera-tunnel.sh
```

Open [the local live feed](http://127.0.0.1:8766/) in a browser. The stream
endpoint is `/stream`, and `/snapshot` returns one JPEG frame. Run
`bash deploy/mac/camera-tunnel.sh stop` to close the tunnel. Other trusted
computers can make the same SSH forward to Pi port 8766.

The image currently faces the room. Its position and orientation relative to
the arm have not been calibrated, so it is not yet a robot wrist-coordinate
sensor or an input to autonomous motion.

## Second USB camera

The `w200` webcam (`0c45:1915`) uses the stable path
`/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._w200_SN0001-video-index0`.
It has its own Pi-local `ustreamer` service on port 8768 at 640×480 MJPEG.
The service starts independently of the first camera and retries after an
unplug/replug. Install it from this worktree on the Pi with:

```bash
bash deploy/pi/install-camera-2.sh
systemctl --user status care-reach-camera-2.service
```

The regular dashboard relays the second feed at `/camera2/stream` and
`/camera2/snapshot` alongside the first camera and ultrasonic range display.
The new camera has not been calibrated into the robot's coordinate frame.
