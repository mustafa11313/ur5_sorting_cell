#!/usr/bin/env bash
# One-shot setup on Ubuntu 22.04 with ROS 2 Humble already installed.
# Installs the dependencies, builds the workspace this repo lives in and runs the tests.
#
#   mkdir -p ~/sorting_ws/src && cd ~/sorting_ws/src
#   git clone <this repo> ur5_sorting_cell      (or unzip it here)
#   bash ur5_sorting_cell/scripts/setup_humble.sh
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$(cd "$REPO/../.." && pwd)"
if [ "$(basename "$(dirname "$REPO")")" != "src" ]; then
  echo "Put the repo inside <workspace>/src first (found it at $REPO)." >&2
  exit 1
fi
set +u; source /opt/ros/humble/setup.bash; set -u

echo "== apt packages"
sudo apt-get update
sudo apt-get install -y python3-colcon-common-extensions python3-rosdep python3-pip \
  ros-humble-moveit ros-humble-pilz-industrial-motion-planner \
  ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gz-ros2-control \
  ros-humble-ur-description ros-humble-ros-gz ros-humble-py-trees ros-humble-vision-msgs

echo "== rosdep"
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
rosdep update
rosdep install --from-paths "$WS/src" --ignore-src -y

echo "== python packages (CPU torch; see the README for CUDA)"
pip3 install --user --index-url https://download.pytorch.org/whl/cpu torch torchvision
pip3 install --user "ultralytics==8.3.40" "numpy<2" "setuptools<70"

echo "== build"
cd "$WS"
colcon build --symlink-install
set +u; source install/setup.bash; set -u

echo "== tests"
colcon test --packages-select sorting_cell_description sorting_cell_perception \
  sorting_cell_task sorting_cell_bringup
colcon test-result --verbose

cat <<MSG

Done. Run the cell with:
  source $WS/install/setup.bash
  ros2 launch sorting_cell_bringup sorting_cell.launch.py
MSG
