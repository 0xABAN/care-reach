"""Control software for the Care Reach SO-101 follower arm.

Subpackages are organized by responsibility, from pure planning outward to
the programs that own the servo bus:

- ``robot``: facts about this one assembled arm and its record files.
- ``motion``: safety bounds and trajectory planning; never touches hardware.
- ``policy``: adapters from model (VLA) actions to validated tick poses.
- ``hardware``: guarded controllers that open the servo bus and move the arm.
- ``routines``: command-line motions (home, park, dance) built on the above.
- ``dashboard``: the local jog panel served on the robot host.

Only one program may own the servo bus at a time.
"""
