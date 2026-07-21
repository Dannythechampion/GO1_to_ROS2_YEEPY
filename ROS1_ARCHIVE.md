# ROS1 archive relationship

The original Go1 ROS1 source snapshot is stored in the separate
`GO-_project_data` repository. Its immutable migration baseline is:

```text
URL: https://github.com/Dannythechampion/GO-_project_data.git
COMMIT: f18fa0fe1f9e6cdcdabb83e89b628b9bb7ad7b40
```

Only calibration, network values, maps, Unitree SDK baseline and documented
robot-specific changes are migrated from that archive.

The archived Unitree SDK baseline is `unitree_legged_sdk v3.8.6`. Its ARM64
library and Python wrapper sources must be rebuilt together for Python 3.10;
files from the v3.5.1 SDK line must never be mixed into this baseline.
