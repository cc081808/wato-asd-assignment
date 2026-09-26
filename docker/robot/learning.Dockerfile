# Reuse the assignment's installed dependencies for a quick local build.
FROM ghcr.io/watonomous/wato_asd_training/robot:main
USER root
WORKDIR /workspace
COPY src/robot src
RUN . /opt/ros/humble/setup.sh && colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
COPY scripts/learning-entrypoint.sh /learning-entrypoint.sh
ENTRYPOINT ["bash", "/learning-entrypoint.sh"]
CMD ["ros2", "launch", "bringup_robot", "robot.launch.py"]
