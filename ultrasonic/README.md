# USB Arduino ultrasonic ranger

For the **UNO Q** and Keyestudio HC-SR04: sensor VCC → Arduino 5V,
GND → Arduino GND, TRIG → D12, ECHO → D13. Connect Arduino USB to the Pi.
Do not connect the sensor's 5 V Echo signal directly to Pi GPIO.

The UNO Q enumerates as USB `2341:0078`. If `robot` cannot upload to it,
install the narrowly scoped rule (the `robot` account is already in `plugdev`):

```bash
sudo install -m 0644 deploy/pi/60-care-reach-uno-q.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add --subsystem-match=usb --attr-match=idVendor=2341 --attr-match=idProduct=0078
```

Replugging the UNO Q after the reload also applies the rule. To upload from
this worktree on the Pi, stop the reader so it releases the serial port:

```bash
systemctl --user stop care-reach-ultrasonic.service
arduino-cli compile --fqbn arduino:zephyr:unoq ultrasonic/keyestudio_ranger
arduino-cli upload -p /dev/ttyACM1 --fqbn arduino:zephyr:unoq ultrasonic/keyestudio_ranger
systemctl --user start care-reach-ultrasonic.service
```

The sketch emits centimeters about every 100 ms at 115200 baud, or `NA` for
no echo. Install the reader with `bash deploy/pi/install-ultrasonic.sh`. It
reconnects after USB unplug/replug and serves status on Pi localhost port 8767.
If the Arduino's port changes, use its current `/dev/serial/by-id/...` path or
set `CARE_REACH_ULTRASONIC_PORT` in the service environment. The reader never
uses the robot's URT-2 serial controller.
