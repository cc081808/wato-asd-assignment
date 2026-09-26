# Cleaner Foxglove recording

Import `config/clean-demo.foxglove.json` through Foxglove's layout menu. Set the app's Appearance to Dark separately; theme is an app preference.

Display settings used:
- Display frame: `sim_world`.
- `/costmap`: hidden. This local grid rotates with the sensor frame and overlaps the persistent grid when both are visible.
- `/map`: visible with Alpha 0.12, making obstacle costs faint enough to see the robot and route.
- `/path` and `/lidar`: visible.
- Camera distance 68, target offsets X/Y/Z 0, theta 129.2, phi 5.3.
- Hide the left settings sidebar and press F11 in Chrome for a full-screen presentation. Press F11 again to exit.

These settings change visualization only. The controller still uses the same navigation data and obstacle clearance.

The new demonstration starts from the previous run's final position near (10, -11) and sends a goal at (-10, -11). It is a live autonomous run with the existing accumulated map, not a fresh-map test. The original three-destination recording remains available separately.

OBS uses Scene 3 and Window Capture for the Foxglove Chrome window. Microphone and desktop audio were muted. Use Start Recording, send the destination, wait for arrival and stopping, and use Stop Recording. OBS initially saves MP4 files in the Windows Videos folder; the delivered copy is in this chat's outputs folder.
