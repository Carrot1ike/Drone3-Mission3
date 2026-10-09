#!/usr/bin/env bash
# Shared CrazySim baseline: official ROS 2 Humble packages on Ubuntu 22.04.
set -euo pipefail
DRONE3_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source /etc/os-release
if [[ "$ID:$VERSION_ID" != ubuntu:22.04 ]]; then
    printf 'Ubuntu 22.04가 필요합니다. 현재: %s\n' "$PRETTY_NAME" >&2
    exit 1
fi
if [[ "$EUID" == 0 ]]; then
    printf '일반 사용자 터미널에서 bash로 실행하세요. 필요한 단계에서만 sudo를 사용합니다.\n' >&2
    exit 1
fi
mkdir -p "$DRONE3_ROOT/logs" "$DRONE3_ROOT/build/ros-setup"
exec > >(tee -a "$DRONE3_ROOT/logs/shared-ros-system-install.log") 2>&1
ROS_SOURCE_DEB="$DRONE3_ROOT/build/ros-setup/ros2-apt-source_1.3.0.jammy_all.deb"
ROS_SOURCE_SHA="110b9a462d55252decb8b7c816f61c2ba0d9890ce5fb93ac504e97cae5860d76"
if [[ ! -f "$ROS_SOURCE_DEB" ]]; then
    curl --fail --location --retry 3 --output "$ROS_SOURCE_DEB" \
        'https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.3.0/ros2-apt-source_1.3.0.jammy_all.deb'
fi
printf '%s  %s\n' "$ROS_SOURCE_SHA" "$ROS_SOURCE_DEB" | sha256sum --check
printf '공식 ROS 2 저장소와 Humble·공유 저장소 빌드 의존성을 설치합니다.\n'
sudo -v
sudo dpkg -i "$ROS_SOURCE_DEB"
sudo apt-get update
packages=(
    ros-humble-desktop ros-humble-rmw-fastrtps-cpp
    git python3-dev python3-venv python3-pip
    build-essential cmake pkg-config
    python3-colcon-common-extensions python3-rosdep python3-pytest
    python3-transforms3d libusb-1.0-0-dev libboost-program-options-dev libeigen3-dev
    libgl1 libglfw3
    ros-humble-motion-capture-tracking-interfaces ros-humble-tf-transformations
    ros-humble-ament-lint-auto ros-humble-ament-lint-common
    ros-humble-ament-cmake-pytest ros-humble-ament-cmake-cppcheck
    ros-humble-ament-copyright ros-humble-ament-flake8 ros-humble-ament-pep257
)
# Refuse removals; install only the dependencies needed for this ROS baseline.
sudo apt-get --simulate --no-remove install "${packages[@]}" \
    > "$DRONE3_ROOT/logs/shared-ros-apt-plan.log"
sudo apt-get --yes --no-remove install "${packages[@]}"
# ROS setup files are not compatible with shell nounset in every distribution.
set +u
source /opt/ros/humble/setup.bash
set -u
python3 -c 'import rclpy; from sensor_msgs.msg import Image; print("ROS 2 Humble Python imports: PASS")'
command -v colcon
command -v rosdep
touch "$DRONE3_ROOT/build/ros-setup/system-install-complete"
printf '\nROS 2 시스템 설치 완료. 공유 저장소의 venv·펌웨어·ROS workspace 빌드를 이어갈 수 있습니다.\n'
