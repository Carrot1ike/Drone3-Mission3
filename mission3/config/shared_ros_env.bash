# Source this file in Bash to select the team ROS 2 SIL environment.
if [[ -n "${VIRTUAL_ENV:-}" ]] && declare -F deactivate >/dev/null; then
    deactivate
fi
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH
unset ROS_PACKAGE_PATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
unset VIRTUAL_ENV LD_LIBRARY_PATH
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
DRONE3_PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
export CRAZYSIM_ROOT="$DRONE3_PROJECT_ROOT/src/crazyflie-ros2-sil"
export ROS_DOMAIN_ID=25 RMW_IMPLEMENTATION=rmw_fastrtps_cpp PYTHONNOUSERSITE=1
source "$CRAZYSIM_ROOT/config/crazysim_env.bash" || return
export ROS_LOG_DIR="$DRONE3_PROJECT_ROOT/logs/shared-ros-manual"
