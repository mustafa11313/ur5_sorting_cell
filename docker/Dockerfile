# UR5 sorting cell: ROS 2 Humble + Gazebo Fortress + MoveIt 2 + YOLO (CPU by default).
#   docker compose -f docker/compose.yaml build
#   docker compose -f docker/compose.yaml run --rm cell
FROM osrf/ros:humble-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
# CPU wheels keep the image small; use https://download.pytorch.org/whl/cu121 for an NVIDIA GPU.
ARG TORCH_INDEX=https://download.pytorch.org/whl/cpu

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip python3-colcon-common-extensions python3-rosdep \
      ros-humble-moveit ros-humble-pilz-industrial-motion-planner \
      ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gz-ros2-control \
      ros-humble-ur-description ros-humble-ros-gz ros-humble-py-trees ros-humble-vision-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir --index-url ${TORCH_INDEX} torch torchvision \
 && pip3 install --no-cache-dir "ultralytics==8.3.40" "numpy<2" \
 # torch/ultralytics pull in a newer setuptools, which breaks colcon builds on Humble
 && pip3 install --no-cache-dir "setuptools<70"

WORKDIR /ws
COPY src /ws/src
RUN apt-get update && (rosdep init 2>/dev/null || true) && rosdep update --rosdistro humble \
 && rosdep install --from-paths src --ignore-src -y --rosdistro humble \
 && rm -rf /var/lib/apt/lists/* \
 && . /opt/ros/humble/setup.sh \
 && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
