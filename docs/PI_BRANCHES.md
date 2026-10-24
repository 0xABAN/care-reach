# Working together on the Pi

The running dashboard uses `~/worktrees/care-reach/pi-runtime` on branch `lance/robot-pi`. Leave that checkout on its branch while the service runs. Switching it changes the code the next service restart will execute.

Each teammate can create a separate branch and checkout on the Pi:

```bash
ssh robot@care-reach-robot-pi
cd ~/worktrees/care-reach/pi-runtime
bash deploy/pi/new-worktree.sh alex/camera
```

The command prints the new checkout path. Edit and test there, then publish that branch with `git push -u origin HEAD` from the new checkout. Use your own name or handle before the slash. The Pi uses `push.default=simple`, so a plain `git push` follows the current branch's upstream; it no longer forces every push to `lance/robot-pi`.

A teammate branch does not change the running dashboard until its changes are reviewed, brought into the live checkout, and the service is restarted. Only the live dashboard should own the URT-2 serial port; stop that service before running any other program that opens the bus.
