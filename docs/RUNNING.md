# Run and inspect the robot

## Where the files are

Clone this repository in Linux or Ubuntu through WSL:

```bash
git clone https://github.com/cc081808/wato-asd-assignment.git
cd wato-asd-assignment
```

While the repository is private, cloning requires an authorized GitHub account. Prerequisites are Git and Docker with Docker Compose. ROS dependencies are provided by the container images.

For the existing checkout on the demonstration machine, open Ubuntu through WSL and enter:

```bash
cd /mnt/c/Users/caden/Documents/Codex/2026-09-25/can/outputs/wato-asd-training
```

All commands below run in that Ubuntu terminal, from this project directory. Docker must be running. PowerShell uses different path and quoting rules, so do not paste these commands into PowerShell unchanged.

## Build and start the isolated simulation setup

```bash
docker compose -f compose.learning.yaml build robot
docker compose -f compose.learning.yaml up -d
docker compose -f compose.learning.yaml ps
```

The first command compiles the C++. The second starts the programs in the background. The third reports whether they are running. Downloading the official images may be necessary on another computer.

This setup uses its own ROS domain, network, Gazebo partition, and Foxglove port so it can coexist with the earlier assignment. Do not mix its port or topics with the original simulation.

To see robot logs:

```bash
docker compose -f compose.learning.yaml logs --tail 60 robot
```

To stop only this setup:

```bash
docker compose -f compose.learning.yaml down
```

To restart with an empty map and the original starting pose:

```bash
docker compose -f compose.learning.yaml down
docker compose -f compose.learning.yaml up -d
```

## Connect Foxglove

1. Open Foxglove, using its web app or your installed desktop app.
2. Open a connection and choose **Foxglove WebSocket**.
3. Enter `ws://localhost:8766`.
4. Import the local layout file `config/learning.foxglove.json` through the layout menu.
5. In the 3D panel, use `sim_world` as the display frame.
6. Enable `/lidar`, `/costmap`, `/map`, and `/path` as needed. Showing only one grid at a time makes it easier to understand.

The bridge provides a WebSocket endpoint. Opening port 8766 as an ordinary webpage will not display Foxglove. The protocol is `ws://`, not `https://`, for this local connection.

The web app may require signing in. Local visualization does not require uploading the recording or publishing the code. If the embedded browser cannot reach the local connection, use your regular browser or Foxglove desktop and the same endpoint.

### What to look for

| View | What it proves |
|---|---|
| Raw `/lidar` | Sensor messages are arriving |
| Local `/costmap` | Returns have become obstacles and clearance zones |
| Global `/map` | Observations remain aligned with the world |
| `/path` | The planner has produced waypoints |
| Raw `/cmd_vel` | The controller is requesting motion or a stop |
| Raw `/odom/filtered` | The simulated position changes as the robot moves |

Use the 3D panel's **Publish point** tool to send a point to `/goal_point` in `sim_world`. Its message type is `geometry_msgs/msg/PointStamped`. A pose published to `/goal_pose` is a different topic and will not activate this planner.

Good initial destinations to try are `(-7, 10)` and `(10, 10)`. Start with one, wait for arrival, then send the next. Avoid clicking inside the obstacle clearance zones.

Do not drive with Teleop while the autonomous controller is running: both would publish movement commands. Use the original warm-up setup for manual driving, or stop the autonomous robot service before a deliberate teleoperation exercise.

## Send a precise goal from the terminal

### Inspect the included recording without running the robot

In Foxglove choose **Open local file(s)** and select `evidence/rosbag-demo/asd-learning-demo_0.db3`. Import `config/learning.foxglove.json` and press Play. The recording contains the LiDAR, maps, route, position, and movement commands; it does not include the camera stream, so that panel will have no image.

The local recording is approximately 140 MiB. Keep its `metadata.yaml` beside it for ROS command-line playback. The separate `evidence/telemetry-replay.gif` is a much smaller visual summary of the earlier three-goal run and can be viewed without Foxglove.

The embedded browser initially failed live connection negotiation and automated local file selection. Live visualization was subsequently verified and screen-recorded in regular Chrome at `ws://127.0.0.1:8766`. Offline bag playback remains unverified. For the cleaner live layout and recording setup, see [CLEAN-RECORDING.md](CLEAN-RECORDING.md).

### Publish a live goal

```bash
docker compose -f compose.learning.yaml exec robot \
  bash /learning-entrypoint.sh ros2 topic pub --once /goal_point \
  geometry_msgs/msg/PointStamped \
  '{header: {frame_id: sim_world}, point: {x: -7.0, y: 10.0, z: 0.0}}'
```

`docker ... exec robot` runs a command inside the running robot container. The entrypoint script loads the ROS environment. `ros2 topic pub --once` sends one message. The quoted braces describe its fields.

## Run the checks

### Start with the warm-up

```bash
docker compose -f compose.learning.yaml exec robot \
  bash /learning-entrypoint.sh ros2 run navigation_tests warmup_publisher
```

Open a Raw Messages panel for `/test_topic`. You should see `Hello, ROS 2!` every half second. Press Ctrl+C in that terminal to end the warm-up. This example sends no movement commands. Its complete source is `src/robot/navigation_tests/warmup_publisher.cpp`.

### Test the navigation

Algorithm checks, without needing a simulator:

```bash
docker compose -f compose.learning.yaml run --rm robot \
  ros2 run navigation_tests navigation_checks
```

End-to-end navigation, with all services running:

```bash
docker compose -f compose.learning.yaml exec robot \
  bash /learning-entrypoint.sh python3 /assignment-scripts/check_navigation.py \
  --goals='-7,10;10,10;10,-11' --timeout 180
```

The probe records received odometry and reports whether each destination was reached. Its output defaults to `/tmp/navigation-run.json` inside the robot container. To keep it locally:

```bash
mkdir -p evidence
docker compose -f compose.learning.yaml cp \
  robot:/tmp/navigation-run.json evidence/my-navigation-run.json
```

Only run one goal-sending tool at a time. A Foxglove click during the test replaces its current destination.

## Edit, build, repeat

1. Change one source file or YAML setting.
2. Rebuild: `docker compose -f compose.learning.yaml build robot`.
3. Restart the changed service: `docker compose -f compose.learning.yaml up -d robot`.
4. Read its logs and repeat the relevant check.

The robot image copies source at build time. The running binary does not automatically update when you save a file. Recreating the robot service also resets its map memory.

## Upstream watod alternative

The official workflow is still available:

```bash
bash ./watod build
bash ./watod up
```

The included `watod-config.sh` selects `robot gazebo vis_tools` and separate local ports. This route builds more dependencies from source and may take longer. Stop the convenience Compose setup before using it. The verification report distinguishes the tested convenience route from any untested build path.

## Troubleshooting

| Problem | What to check |
|---|---|
| Shell reports `\r` or “bad interpreter” | Script has Windows CRLF endings; save as LF. `.gitattributes` documents the rule. |
| Foxglove connection refused | Check `docker compose ... ps`, bridge logs, and port 8766. |
| Topics exist but no LiDAR arrives | Inspect Gazebo logs; GPU LiDAR needs a working render backend even in a headless simulation. |
| Map never appears | Look for `Waiting for scan transform` in robot logs; check TF and incoming scans. |
| Goal does nothing | Confirm `/goal_point`, `PointStamped`, and frame `sim_world`; inspect planner logs. |
| No route | Goal/start may be outside the map or within inflated obstacles. Try an open destination. |
| Robot stops unexpectedly | Check path, sensor and odometry freshness; inspect the forward obstacle guard. |
| Changes have no effect | Rebuild and recreate the robot container. |

## Official references

- [Assignment](https://wiki.watonomous.ca/admission_assignments/asd_admission_assignment/)
- [Starter repository](https://github.com/WATonomous/wato_asd_training)
- [Foxglove Bridge documentation](https://docs.foxglove.dev/docs/fleet/bridge)
