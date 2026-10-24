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

Its position and orientation relative to the arm have not been calibrated.
[Picking up objects](PICK.md) avoids needing that: with the camera on the
wrist mount, it looks and nudges until the object sits where the jaws are, and
a one-time setup records which picture direction is which gripper axis.
