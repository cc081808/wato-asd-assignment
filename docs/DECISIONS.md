# Design choices, challenges, and limitations

## Separate communication from mathematics

Each of the four packages has a `*_node` class for subscriptions, publishers, and timers and a `*_core` class for the algorithm. Core functions can be tested using constructed messages without running Gazebo. This follows the starter's intended organization.

## Use the actual simulation frame

The upstream odometry helper reports the LiDAR pose in `sim_world`. We changed its target to the `robot` model frame, so planning and control track the body/drive reference. The global map, goals, and paths use `sim_world` consistently. Goals in other frames are rejected with a warning. The local grid keeps the incoming scan frame. Map memory independently uses the sensor's TF at scan time, accounting for displacement and rotation.

## Preserve static obstacles conservatively

The tutorial suggests overwriting known observations. This implementation instead keeps the highest observed cost in each global cell. That prevents a later free beam or partial view from erasing a previously inflated obstacle boundary. It is suitable for this static simulated room, but noisy detections can leave persistent marks and moved obstacles will not clear. Restarting map memory resets the map.

A more general mapper would retain raw occupancy evidence separately, clear free space probabilistically, and recompute inflation after fusion. That is a useful extension, not implemented here.

## Update while stationary as well as moving

The assignment's examples contain both 1.5-metre and 5-metre update thresholds. This version uses a 250-ms timer to merge the latest available observation, including while stationary or rotating. This avoids delaying detection until the robot travels a large distance. The initial empty map is published with retained delivery so a newly started planner can receive it.

## Account for a large, offset robot

The simulated chassis is 2 metres long, offset forward from the model frame, and the LiDAR sits farther forward. A 2-metre circular clearance radius conservatively covers the robot about the model reference point, including a margin. The outer 2.6-metre band makes nearby routes more expensive. This sacrifices tight passages for easier, more reliable avoidance.

An initial integration run exposed why the reference point matters: the front-mounted sensor began inside the conservative margin around the central obstacle, although the robot body had room to move. Reporting the model pose solves that mismatch and makes Pure Pursuit's reference consistent with the drive geometry. An explicit oriented footprint could reclaim some of the space lost to circular inflation.

## Unknown is allowed, with a cost

Blocking all unknown space can prevent travel to an unseen goal. We allow unknown cells with a factor-of-two movement cost. Inflated cells below 100 also increase cost; cells at 100 are blocked. The planner searches every 500 ms while a goal is active. It retains an unreachable goal and retries as the map changes; it accepts a replacement goal at any time.

This is basic navigation, not a dedicated exploration strategy. Some unreachable goals will remain waiting indefinitely. Empty paths keep the controller stopped.

## Bounded map

The fixed global map spans approximately `[-20,20)` metres in both directions, enough for the supplied room. Out-of-bounds observations are ignored and out-of-bounds goals yield no route. The map does not grow dynamically.

## Stopping behaviour

The control node publishes a zero command for an empty route, missing input, mismatched path/odometry frames, or input receipt older than 1.5 seconds. It also uses a short forward LiDAR guard. Arrival uses a 0.35-metre tolerance. The planner and controller share that tolerance.

The watchdog observes message receipt, not independent validation of sensor timestamps. A process repeatedly publishing stale measurements can evade it. The forward guard is not a full swept-footprint collision predictor. These are limitations for future work.

## Local environment and reproducibility

`compose.learning.yaml` uses a separate Compose project, ROS domain 67, Gazebo partition, and loopback Foxglove port 8766. This preserves the user's original running assignment. It reuses the official prebuilt runtime images already present locally and builds the new C++ into a new image.

The upstream `watod` build remains available, with separate local project and port names. Use one launch method at a time. The convenience images use the mutable `main` tag; the verification report records the tested environment, but exact long-term reproducibility would require pinning image digests.

## Authorship

This implementation and its documentation were prepared with AI assistance. The original assignment, infrastructure, simulation, and starter layout are WATonomous work. See the root LICENSE and official repository.

## Future improvements

- Probabilistic occupancy updates and proper clearing of moved obstacles.
- Explicit robot footprint and axle-centred controller geometry.
- Progress-based recovery for a robot stuck near an obstacle.
- Dynamic map size and transforms for goals from arbitrary frames.
- Timestamp-based sensor health checks and a swept-path collision check.
- Tuning and performance measurements across more worlds and initial poses.
