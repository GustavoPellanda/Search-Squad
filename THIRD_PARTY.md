# Third-party resources

`src/simulation/models/burger/model.sdf.in` and `src/simulation/urdf/burger.urdf`
derive from ROBOTIS TurtleBot3 simulations, tag **2.3.8**, Apache License 2.0:
https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/2.3.8/turtlebot3_gazebo

Authors credited upstream: Darby Lim, Pyo, Ryan Shim and Hyungyu Kim (ROBOTIS).
Original files: `models/turtlebot3_burger/model.sdf`, `urdf/turtlebot3_burger.urdf`.
The model adds namespaces, explicit sensor frames, 10 Hz LiDAR, matching link poses
and an independent timed encoder drive. The URDF is retained, with attribution.
The Apache-2.0 license is reproduced in `LICENSE`.

Burger meshes are resolved from the declared binary dependency `turtlebot3_gazebo`;
they are not copied, downloaded by the GUI, or obtained from another workspace.

The timed drive, world, controller and command gate are original Search Squad code.
The earlier course workspace remains an independent reference. The old controller was consulted for C++ organization
and LiDAR angular normalization; no source excerpt from it is incorporated.
Nav2 Collision Monitor and Gazebo ROS are declared system dependencies, unmodified.

Original binary-package resource SHA-256 (before adaptation):

- `model.sdf`: `179e9595d4e229cf207c3fc0515aa66f4bc4434a37b398fc3c39753dc2feffb1`
- `turtlebot3_burger.urdf`: `e1f4671597b3e578ba31afc492773de31b7e516636287aa152589234de6226e3`

`collision_monitor_runner` links the unmodified official Nav2 CollisionMonitor
class. Its small main function is original; no upstream library source is copied.

`src/indoor_navigation/config/nav2.yaml` adapts the author's classroom
`nav2_integration` configuration, originally based on ROBOTIS
`turtlebot3_navigation2/param/humble/burger.yaml` (Apache-2.0):
https://github.com/ROBOTIS-GIT/turtlebot3/tree/humble/turtlebot3_navigation2/param/humble
The independent copy uses project frames, sensor topics, conservative velocity
limits, live map updates and room keepout filters. No course overlay is required.
Nav2 servers and plugins remain unmodified system dependencies.
The project-owned `LiveObstacleLayer` subclasses Nav2's `ObstacleLayer` and
changes scan scheduling; it does not vendor or patch upstream library sources.
Nav2's Regulated Pure Pursuit controller is also used as an installed plugin:
https://github.com/ros-navigation/navigation2/tree/humble/nav2_regulated_pure_pursuit_controller

`shared_mapping` launches the unmodified installed SLAM Toolbox Humble executable
(`async_slam_toolbox_node`). Its parameter file adapts the upstream Humble example:
https://github.com/SteveMacenski/slam_toolbox/blob/humble/config/mapper_params_online_async.yaml
SLAM Toolbox is maintained by Steve Macenski and contributors; the installed
2.6.10 package declares LGPL. Upstream files retain their own license notices:
https://github.com/SteveMacenski/slam_toolbox/tree/humble
No SLAM implementation is copied or patched. Project configuration sets robot
frames, simulation time, sensor range, map update rate and motion thresholds.
The parallel profile runs three independent installed instances with separate
maps and adjusted encoder-prior penalties. `slam_frontend` (measured initial
alignment and peer masking) and `map_fusion` (replaceable grid composition) are
original Search Squad nodes. The invalid-range behavior was checked against
the upstream Humble Karto implementation; no Karto source is incorporated:
https://github.com/SteveMacenski/slam_toolbox/blob/humble/lib/karto_sdk/include/karto_sdk/Karto.h
Nav2 map saving and ROS 2 teleop_twist_keyboard are unmodified installed tools.
