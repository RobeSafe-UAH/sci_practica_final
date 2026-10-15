# RoboRacer F1TENTH visual models

Source: [RobeSafe-UAH/rbsf-racing](https://github.com/RobeSafe-UAH/rbsf-racing),
commit `79a61ea7c14172b5bd736b0447b58c75da19c7af`.

The six `roboracer_*.glb` files and `roboracer_accent.stl` are unmodified copies
of `src/base_system/f1tenth_gym_ros/meshes/`. They are self-contained local
assets: no download, ROS package URI, Gym runtime, or RViz is required.

The source package declares the MIT license, Copyright (c) 2020 Hongrui Zheng.
The original complete license is included as `LICENSE` in this directory.

The MVSim visual poses in `definitions/ackermann.vehicle.xml` and
`definitions/traffic.block.xml` follow
`src/base_system/f1tenth_gym_ros/urdf/racecar_mesh.xacro`: F1TENTH scale 1,
rear-axle origin, original LiDAR/wheel translations, and per-wheel roll,
pitch and yaw alignment. The original alignment values are already in degrees;
MVSim uses degrees too. The chassis and accent require no translation.

SCI changes: direct MVSim visual entries, blue/orange accent colors, and local
relative asset paths. Mesh wheels are fixed visual parts of the chassis;
MVSim retains its own Ackermann wheel dynamics and explicit collision shape.
