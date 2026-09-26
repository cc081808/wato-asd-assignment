# Local ROS recording

The full SQLite ROS bag is retained in the local project folder. It is excluded from Git because it is approximately 140 MiB. The MP4 screen recordings are also kept outside the repository.

The accompanying metadata describes that local recording; metadata alone is not a playable bag. The repository includes the smaller telemetry results, trajectory image, and animated replay in the parent evidence directory.

To generate a new recording, follow the project setup instructions and run `scripts/record_demo.py` inside the robot container. This script sends a destination and records live topics; read it before running to confirm the destination and output path.
