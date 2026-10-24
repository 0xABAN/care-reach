# Pi camera live feed

The connected USB camera is `Innomaker-U20CAM-720P` (`0c45:6367`). It provides
MJPEG at 1280×720 and 30 fps. Use the stable capture device path
`/dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-720P_SN0001-video-index0`.
The Pi's user service runs `ustreamer` on **Pi localhost only**, port 8766.
It does not own or change the robot's serial bus.

On the Pi, install the Debian package and user service:

```bash
sudo apt-get install -y ustreamer
bash deploy/pi/install-camera.sh
systemctl --user status care-reach-camera.service
```

On a Mac that can SSH to the Pi over Tailscale:

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
