# Test results

Tested locally on September 25–26, 2026, using WSL 2, Docker, and ROS 2 Humble in the assignment containers. The tested launch method was `compose.learning.yaml`. The alternative `watod` build was not separately tested end to end.

## Build and basic checks

- All seven ROS packages built: [build log](../evidence/build.log).
- All 20 algorithm checks passed: [test output](../evidence/algorithm-checks.txt). These cover LiDAR readings, mapping, route planning, and control.
- A separate subscriber received three warm-up messages: [warm-up output](../evidence/warmup-check.txt).

## Driving and stopping

In the first three-goal test, the robot reached `(-7, 10)`, `(10, 10)`, and `(10, -11)`. The test reports arrival within 0.4 metres; the controller keeps going until it is within 0.35 metres. It finished 0.330 metres from the last goal with zero movement commands.

- [Recorded positions and goal results](../evidence/navigation-run.json)
- [Short result log](../evidence/simulation-checks.txt)

A fourth trip reached `(-10, -11)` and stopped. A goal at `(0, 0)`, inside the central obstacle, kept the robot stopped throughout 131 command observations: [stopping results](../evidence/stopping-check.json).

A check of the recorded positions found room between the robot and the obstacles: [clearance results](../evidence/trajectory-summary.json). This checks sampled positions, so it is not proof that every instant was collision-free.

## Foxglove and videos

Foxglove connected successfully in Chrome at `ws://localhost:8766`, and OBS recorded the live runs. The first video reached three goals, but its test script timed out on the second goal while the robot was still moving. The script was restarted with a longer timeout and the remaining goals succeeded: [first part](../evidence/recording-first-goals.json), [continuation](../evidence/recording-continuation.json).

The cleaner video shows a trip from near `(10, -11)` to `(-10, -11)`, using the map already built by the previous run. Arrival and a final zero movement command were confirmed. [Clean run data](../evidence/clean-demo.json).

The MP4 videos and full ROS recording are kept locally, outside the Git upload. [Recording metadata and instructions](../evidence/rosbag-demo/README.md) explain the missing `.db3` file. Offline playback of that file in Foxglove was not verified.

## What remains untested

The tests cover the supplied simulated room. Separate live tests that deliberately cut off sensor data were not performed. Moving obstacles, other worlds, and all possible stuck situations have not been tested.
