# Lembretes:

- A navegação do scout ainda não considera curvas. Se der tempo, eu tenho que fazer algum sistema para diferenciar um buraco de porta de uma curva no corredor e fazer o robô virar quando encontrar uma parede de fundo. Por enquanto, também não tem nada para o caso de ter obstáculos no corredor.

# Perception node:

### corridor_perception.cpp

- Subscribes to `/scout/scan` (`sensor_msgs/msg/LaserScan`).
- Receives transforms through the TF listener on `/tf` and `/tf_static`, and simulation time from `/clock`.

`scan_callback()` stores the latest scan. Every 50 ms, `process_scan()` checks its metadata and freshness, processing each acquisition only once. Invalid scans or readings older than 0.3 seconds produce an invalid observation. Missing TF is retried while the data remains fresh.

As the robot moves through the corridor, the LiDAR measures distances at different beam angles. The distances come from the sensor; TF provides the robot and sensor poses in `scout/odom` at the time of the scan. `scout/odom` is a reference frame here; the node does not subscribe to `/scout/odom`.

The angle and distance are converted by `project_scan_to_odom()` into `(x, y)` coordinates using `x = distance * cos(angle)` and `y = distance * sin(angle)`, then transformed into odom coordinates. It also measures clearance ahead of the footprint; no usable frontal return means unavailable clearance. `interpret_scan()` passes this vector of points to `CorridorGeometry::estimate()`.

`make_observation()` converts the returned geometry to the scan-time robot base frame, with x forward and y left. The message distinguishes valid inputs, a valid corridor axis, and measured or predicted walls.

- Publishes measurements to `/scout/corridor_observation` (`squad_interfaces/msg/CorridorObservation`).
- Publishes RViz markers to `/scout/corridor_markers` (`visualization_msgs/msg/MarkerArray`).

### corridor_geometry.cpp

This file implements a geometry library used by `corridor_perception`, not another ROS node. It has no subscriptions or publishers.

The `extract_segments()` method groups consecutive points, separating them at gaps or invalid readings. It calls `split_segment()` to fit a straight line and measure the points' deviation from it. If the deviation is too large, the group is split and checked again. Accepted segments need enough points and length.

Each accepted segment is stored as a `LineSegment`, containing its endpoint coordinates, center, angle, length and fitting error. The `associate_walls()` method selects approximately parallel segments belonging to the corridor walls on the left and right of the robot.

The robot's heading starts the search direction. The measured walls then refine the corridor heading, with longer segments contributing more. A valid axis requires both walls to be available and their separation to be within the configured width limits.

The `calculate_axis()` method calculates the perpendicular distances to those walls:

```cpp
if(left_available) estimate.left_distance = left - position;
if(right_available) estimate.right_distance = position - right;
```
Here, `left`, `right` and `position` are the wall and robot positions projected perpendicular to the corridor. The corridor width is `left - right`, and its central axis lies halfway between the walls. The lateral error measures the robot's displacement from that axis, positive to the left. The angular error is the robot's heading minus the corridor's heading.

These measurements return to `corridor_perception` for publication and will later be used by the fuzzy system to guide the Scout. For now, they do not change its movement.


Door openings: `find_openings()` marks gaps between 0.60 and 2.00 m between measured segments of the same wall as opening candidates. They are not confirmed doors. A missing wall can be marked as predicted using its last observation, limited to 20 seconds and 1.5 m of displacement from that observation's robot position.

Back wall: `terminal_wall_evidence()` looks for approximately perpendicular segments ahead of the robot. If aligned fragments cover more than 75% of the valid corridor width, they are marked as a back-wall candidate. This does not yet make the Scout stop or turn.
