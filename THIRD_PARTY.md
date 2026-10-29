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
No course package was copied. The old controller was consulted for C++ organization
and LiDAR angular normalization; no source excerpt from it is incorporated.
Nav2 Collision Monitor and Gazebo ROS are declared system dependencies, unmodified.

Original binary-package resource SHA-256 (before adaptation):

- `model.sdf`: `179e9595d4e229cf207c3fc0515aa66f4bc4434a37b398fc3c39753dc2feffb1`
- `turtlebot3_burger.urdf`: `e1f4671597b3e578ba31afc492773de31b7e516636287aa152589234de6226e3`

`collision_monitor_runner` links the unmodified official Nav2 CollisionMonitor
class. Its small main function is original; no upstream library source is copied.
