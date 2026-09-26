# Design notes

## Keeping the map lined up

LiDAR readings start relative to the sensor. Map memory uses the sensor's position and rotation to place them in the world map. The map, goals, and route all use the same frame, `sim_world`.

The position helper was changed to report the robot body's position instead of the front-mounted LiDAR's position. This gives the controller a more suitable point to steer from.

## Leaving room around obstacles

The robot is large, so planning a line that just misses an obstacle is not enough. The costmap adds a 2-metre blocked area around each obstacle, with an extra band out to 2.6 metres that makes nearby routes less attractive.

This is a simple way to leave room for the body, but it also rules out some narrow gaps the robot might otherwise fit through.

## Remembering obstacles

Map memory keeps the highest obstacle value seen in each square. This helps avoid accidentally erasing an obstacle after the robot passes it. The map updates every 0.25 seconds, even when the robot is stopped.

The downside is that false detections and moved obstacles leave marks behind. Restarting the robot service clears the map. The map also has fixed limits of about -20 to 20 metres in each direction.

## Finding and following a route

A* searches the grid for a route and avoids cutting diagonally between blocked corners. Unknown areas are allowed but cost more to travel through. The planner checks for a new route every 0.5 seconds while a goal is active.

Pure Pursuit steers towards a point 0.65 metres ahead on the route. The maximum speed is 0.6 m/s, and the controller stops within 0.35 metres of the goal. These values are in each package's `config/params.yaml`.

## Stopping and limitations

An empty route or missing input makes the controller send a stop command. It also stops if required messages have not arrived for 1.5 seconds, and checks for obstacles directly ahead using LiDAR.

These checks have limits: repeated old messages can still appear current, and the forward check does not cover the robot's full turning motion. Some unreachable goals may leave it waiting indefinitely. The tests do not cover every possible failure.

## Possible improvements

- Clear old obstacle marks when an area becomes empty.
- Use the robot's actual shape instead of a circle for clearance.
- Add a way to recover when the robot gets stuck.
- Test more starting positions and environments.

The project follows the starter's split between ROS communication (`*_node`) and calculations (`*_core`). AI assistance was used for implementation and documentation; the starter and simulation are WATonomous work.
