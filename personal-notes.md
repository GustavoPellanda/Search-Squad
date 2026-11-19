# Lembretes:

- A navegação do scout ainda não considera curvas. Se der tempo, eu tenho que fazer algum sistema para diferenciar um buraco de porta de uma curva no corredor e fazer o robô virar quando encontrar uma parede de fundo. Por enquanto, também não tem nada para o caso de ter obstáculos ou bifurcações no corredor.

- Fica legal criar uma central de resultados no do final da missão e colocar umas coisas visuais para fazer análises. Fica legal colocar um comparativo dos mapas feitos por cada robô individualmente com sobreposição ao ground truth do Gazebo com o mapa compartilhado. Que sabe as rotas que cada robô fez também. Pensar se tem mais alguma coisa. O mapa de entradas detectadas pelo scout fica legal também.

- Às vezes as rotas dos explorers não ficam otimizadas porque o scout deixa uns pequenos espaços não mapeados por onde ele passou. Não decidi o que fazer em relação a isso ainda. 

# System overview:

Every robot in the squad has a base anti-colison system, to prevent them from crashing into walls or obstacles. Using their Lidar sensors, they will stop if they're too close from an object in front of them. {Package: Lidar Safety}

The execution of the mission starts with the Scout. This robot will have to navigate through an unknown environment, and uses a fuzzy control system to do so. The scout percieves the environment through it's lidar sensors. The points read by the sensors are used by a perception node to form straight lines, that can be interpreted as walls. Parelel walls, in a small distance from each other, make up a corridor. {Package: Perception}

A Fuzzy control system uses the detected walls to guide the Scout throgh corridors. Considering the distance between walls, how well centered the robot is and if it points in the same direction as the corridor, the fuzzy sistem makes the robot turn left or right. It also detects a back wall at the end of the corridor (explain how) and stops it when the corridor ends. {Package: Scout}

The scout creates a map... 

Dispatcher sends explores to the rooms... (finish)


# Lidar Safety package:

### lidar_safety node:


# Perception package:

### corridor_perception node:

- Subscribes to `/scout/scan` (`sensor_msgs/msg/LaserScan`).
- Receives transforms through the TF listener on `/tf` and `/tf_static`, and simulation time from `/clock`.

`scan_callback()` stores the latest scan. Every 50 ms, `process_scan()` checks its metadata and freshness, processing each acquisition only once. Invalid scans or readings older than 0.3 seconds produce an invalid observation. Missing TF is retried while the data remains fresh.

As the robot moves through the corridor, the LiDAR measures distances at different beam angles. The distances come from the sensor; TF provides the robot and sensor poses in `scout/odom` at the time of the scan. `scout/odom` is a reference frame here; the node does not subscribe to `/scout/odom`.

The angle and distance are converted by `project_scan_to_odom()` into `(x, y)` coordinates using `x = distance * cos(angle)` and `y = distance * sin(angle)`, then transformed into odom coordinates. It also measures clearance ahead of the footprint; no usable frontal return means unavailable clearance. `interpret_scan()` passes this vector of points to `CorridorGeometry::estimate()`.

`make_observation()` converts the returned geometry to the scan-time robot base frame, with x forward and y left. The message distinguishes valid inputs, a valid corridor axis, and measured or predicted walls.

These are the measurements teh perception node may publish:

| Field | Meaning |
| --- | --- |
| `width` | Estimated corridor width, in metres. |
| `lateral_error` | Robot offset from the corridor centre, in metres. Positive means left of centre. |
| `heading_error` | Robot heading minus corridor heading, in radians. |
| `axis` | A point on the corridor centreline and its orientation relative to the robot. |
| `left_wall_distance` / `right_wall_distance` | Perpendicular distance to each side wall, in metres. |
| `front_clearance` | Free distance ahead of the robot's front edge within the frontal band, in metres. |
| `left_wall_age` / `right_wall_age` | Seconds since each wall was last measured. |
| `confidence` | Support-based confidence from 0 to 1; not a calibrated probability. |
| `left_segments` / `right_segments` | Measured wall segments, stored as consecutive pairs of endpoints in the robot base frame, in metres. |
| `left_openings` / `right_openings` | Candidate gaps, stored as consecutive pairs of endpoints in the same frame. Gap width is the distance between its endpoints. |
| `terminal_wall_candidate` | Evidence of a wall closing the corridor; not confirmation that the mission has finished. |

- Publishes measurements to `/scout/corridor_observation` (`squad_interfaces/msg/CorridorObservation`).
- Publishes RViz markers to `/scout/corridor_markers` (`visualization_msgs/msg/MarkerArray`).

### corridor_geometry node:

This file implements a geometry library used by `corridor_perception`, not another ROS node. It has no subscriptions or publishers.

The `extract_segments()` method groups consecutive points, separating them at gaps or invalid readings. It calls `split_segment()` to fit a straight line and measure the points' deviation from it. If the deviation is too large, the group is split and checked again. Accepted segments need enough points and length.

Each accepted segment is stored as a `LineSegment`, containing its endpoint coordinates, center, angle, length and fitting error. The `associate_walls()` method selects approximately parallel segments belonging to the corridor walls on the left and right of the robot.

The robot's heading starts the search direction. The measu  red walls then refine the corridor heading, with longer segments contributing more. A valid axis requires both walls to be available and their separation to be within the configured width limits.

The `calculate_axis()` method calculates the perpendicular distances to those walls:

```cpp
if(left_available) estimate.left_distance = left - position;    
if(right_available) estimate.right_distance = position - right;
```
Here, `left`, `right` and `position` are the wall and robot positions projected perpendicular to the corridor. The corridor width is `left - right`, and its central axis lies halfway between the walls. The lateral error measures the robot's displacement from that axis, positive to the left. The angular error is the robot's heading minus the corridor's heading.

These measurements return to `corridor_perception` for publication and are used by the fuzzy system to guide the Scout. The coordinates of the walls found are not stored in a map or any permaent file at this point. The Scout guidance system only keeps track of the two walls that are on each side of it.

Door openings: `find_openings()` marks gaps between 0.60 and 2.00 m between measured segments of the same wall as opening candidates. They are not confirmed doors. A missing wall can be marked as predicted using its last observation, limited to 20 seconds and 1.5 m of displacement from that observation's robot position.

Back wall: `terminal_wall_evidence()` looks for approximately perpendicular segments ahead of the robot. If aligned fragments cover more than 75% of the valid corridor width, they are marked as a back-wall candidate. This does not yet make the Scout stop or turn.


# Scout Package:

### corridor_follower node:

### fuzzy_steering node:

### entrance_detector node:

As the Scout moves, the Perception package detects walls on both sides of it. It also finds gaps on these walls and publishes them in `/scout/corridor_observation` with a timestamp. The entrance detector subscribes to that topic and, when a new gap is published, its task is to verify if it is an entrance to a room. It also subscribes to `/scout/scan` and keeps a short queue of scans, so it can check the scan with that same timestamp.

First, it checks that the data is valid and the wall was actually measured, rather than predicted from previous readings. It uses the raw LiDAR points near the wall to find the two edges of the opening more precisely, since fitting the wall segments can leave out points near the corners. These edges give the opening's center and width, and the wall's direction gives a perpendicular direction pointing into the room.

At least three LiDAR beams must pass through the opening, keeping 10 cm away from each edge, and show at least 25 cm of free space into the room. Invalid readings do not count. The node also checks `/map`: occupied space across the opening can reject it, while unknown cells are treated as missing information.

The opening must appear in at least four different scans, and the Scout must move at least 8 cm from where it first observed it. The node compares position, width and inward direction in `scout/odom` to recognize the same opening as the robot moves. Once confirmed, it receives an ID, such as `entrance_1`. Closer views refine the measurements, while distant views after passing the door do not overwrite the better estimate.

The center and inward direction are transformed into `map` using TF at the selected scan's timestamp. The Scout keeps moving through the corridor while the detector registers the entrances. This identifies geometric openings; it does not determine the boundaries of the rooms.

The Scout publishes:

- `/fleet/entrances`: ID, position, orientation, width, observation count and timestamp.
- `/fleet/markers`: a line across the opening, an arrow pointing into the room, and text with the ID, coordinates and width, shown over the map in RViz.
