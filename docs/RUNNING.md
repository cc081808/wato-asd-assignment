# Running the project

Use Linux or an Ubuntu terminal in WSL, with Git and Docker Compose installed and Docker running. These commands use Linux syntax, not PowerShell syntax. ROS runs inside the containers.

## 1. Get the code

```bash
git clone https://github.com/cc081808/wato-asd-assignment.git
cd wato-asd-assignment
```

While the repository is private, you need a GitHub account with access to clone it. If you already have the project, open a terminal in its folder instead.

## 2. Build and start

```bash
docker compose -f compose.learning.yaml build robot
docker compose -f compose.learning.yaml up -d
docker compose -f compose.learning.yaml ps
```

The first command compiles the code. The second starts the simulation, robot code, and Foxglove connection. The third shows whether they are running. The first build may need to download the official assignment images.

## 3. Connect Foxglove

1. Open Foxglove in Chrome.
2. Choose **Foxglove WebSocket** and enter `ws://localhost:8766`.
3. Import `config/clean-demo.foxglove.json` from the layout menu.
4. Check that the 3D display frame is `sim_world`.
5. Use **Publish point** to send a destination on `/goal_point`.

Try an open point such as `(-7, 10)`. The robot should start driving on its own. Do not use the joystick at the same time, because it also sends movement commands.

The goal topic is `/goal_point`, not `/goal_pose`. The message type is `geometry_msgs/msg/PointStamped`.

You can also send that goal from the terminal:

```bash
docker compose -f compose.learning.yaml exec robot \
  bash /learning-entrypoint.sh ros2 topic pub --once /goal_point \
  geometry_msgs/msg/PointStamped \
  '{header: {frame_id: sim_world}, point: {x: -7.0, y: 10.0, z: 0.0}}'
```

`config/learning.foxglove.json` is the more detailed layout. See [recording setup](CLEAN-RECORDING.md) for the cleaner display.

## 4. Run the checks

Algorithm checks:

```bash
docker compose -f compose.learning.yaml run --rm robot \
  ros2 run navigation_tests navigation_checks
```

Three-goal driving test, with all services running:

```bash
docker compose -f compose.learning.yaml exec robot \
  bash /learning-entrypoint.sh python3 /assignment-scripts/check_navigation.py \
  --goals='-7,10;10,10;10,-11' --timeout 600
```

This sends goals and checks the robot's position. Do not send other goals while it runs. The longer timeout allows for a slow simulation.

Warm-up publisher:

```bash
docker compose -f compose.learning.yaml exec robot \
  bash /learning-entrypoint.sh ros2 run navigation_tests warmup_publisher
```

Open a Raw Messages panel on `/test_topic` to see `Hello, ROS 2!`. Press Ctrl+C in the terminal to stop the publisher.

## Stop or rebuild

Stop the simulation:

```bash
docker compose -f compose.learning.yaml down
```

After changing code or settings:

```bash
docker compose -f compose.learning.yaml build robot
docker compose -f compose.learning.yaml up -d robot
```

Saving a file alone does not update the running program. Restarting the robot service clears its remembered map. To reset both the map and robot position, run `down`, then `up -d` for the full setup.

## Common problems

| Problem | What to try |
|---|---|
| Linux paths or commands fail in PowerShell | Open an Ubuntu/WSL terminal instead. |
| Foxglove will not connect | Check the services with `docker compose -f compose.learning.yaml ps` and use port 8766. |
| Goal does nothing | Check `/goal_point` and `sim_world`, then try a point farther from obstacles. |
| Robot stops or no map appears | Read the logs with the command below. |
| Code changes do not appear | Rebuild and restart the robot service. |

```bash
docker compose -f compose.learning.yaml logs --tail 60 robot gazebo foxglove
```

The `.db3` recording is not included in Git. See [recording notes](../evidence/rosbag-demo/README.md) if you want to create a new one.
