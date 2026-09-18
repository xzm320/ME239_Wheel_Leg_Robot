# Official Upkie prototype

This model is the stock wheeled biped from:

- Hardware / software: https://github.com/upkie/upkie
- URDF / Xacro: https://github.com/upkie/upkie_description

Pinned description commit: `94735fbe6137276a41de0ff4cc04d2e533fa9e33` (2025-11-25).

`robot.xml` is that URDF compiled to MJCF (floating base, official joint
limits and motor force ranges). Serial hip → knee → wheel. No extra
compliance, four-bar, or telescopic crossbar.

Apache-2.0; see `LICENSE`.
