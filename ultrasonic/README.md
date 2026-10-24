# USB Arduino ultrasonic ranger

For a **5 V Arduino** and Keyestudio HC-SR04: sensor VCC → Arduino 5V,
GND → Arduino GND, TRIG → D12, ECHO → D13. Connect Arduino USB to the Pi.
Do not connect the sensor's 5 V Echo signal directly to Pi GPIO.

Upload `keyestudio_ranger/keyestudio_ranger.ino` to the Arduino. It emits a
centimeter reading every ~100 ms at 115200 baud, or `NA` for no echo. Then on
the Pi run `bash deploy/pi/install-ultrasonic.sh` from this worktree. The
service keeps running if the Arduino is unplugged and reconnects when it
reappears. Its status is available only on Pi localhost port 8767. If there
are multiple Arduino-style USB devices, set `CARE_REACH_ULTRASONIC_PORT` in
the service environment to the correct `/dev/serial/by-id/...` path.
