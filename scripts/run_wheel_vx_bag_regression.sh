#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PKG_DIR=$(cd "$SCRIPT_DIR/.." && pwd)
WS=${WS:-$(cd "$PKG_DIR/../.." && pwd)}

BAG=${1:-/home/guoli/data/yangpu/0713/v227_2026-07-13-16-12-32.bag}
OUTPUT_BAG=${2:-$PKG_DIR/test_out/wheel_vx_ekf_2800_100.bag}
SKIP=${3:-2800}
DURATION=${4:-100}
RATE=${5:-1.0}
WHEEL_VARIANCE=${WHEEL_VARIANCE:-0.05}
WHEEL_ENABLED=${WHEEL_ENABLED:-true}
WHEEL_BLEND=${WHEEL_BLEND:-0.25}
WHEEL_GATE=${WHEEL_GATE:-0.03}
REPLACE_LIO_VX=${REPLACE_LIO_VX:-false}
ROS_MASTER_PORT=${ROS_MASTER_PORT:-11337}

# roslaunch changes each node's working directory to ROS_HOME. Always pass an
# absolute recorder path so a caller-provided relative path cannot resolve in
# that temporary directory and fail after playback has already started.
if [[ "$OUTPUT_BAG" != /* ]]; then
  OUTPUT_BAG="$PWD/$OUTPUT_BAG"
fi

if [[ ! -f "$BAG" ]]; then
  echo "[ERROR] Input bag not found: $BAG" >&2
  exit 2
fi
if [[ -e "$OUTPUT_BAG" || -e "${OUTPUT_BAG%.bag}.bag.active" ]]; then
  echo "[ERROR] Output already exists; choose a new path: $OUTPUT_BAG" >&2
  exit 2
fi

mkdir -p "$(dirname "$OUTPUT_BAG")"
RUN_STEM=${OUTPUT_BAG%.bag}
# Use per-run ROS directories. Reusing one directory makes roslaunch scan every
# previous run's logs before starting, which can add minutes to parameter sweeps.
RUN_TAG=$(basename "$RUN_STEM")_$$
export ROS_HOME=${ROS_HOME:-/tmp/robot_loc_wheel_vx_ros_home_${RUN_TAG}}
export ROS_LOG_DIR=${ROS_LOG_DIR:-/tmp/robot_loc_wheel_vx_ros_logs_${RUN_TAG}}
export ROS_MASTER_URI="http://127.0.0.1:${ROS_MASTER_PORT}"
export ROS_IP=127.0.0.1
export ROS_HOSTNAME=127.0.0.1
# XML-RPC is strictly local for this regression. Proxy variables can make ROS
# command-line clients hang while trying to reach the local master via HTTP.
unset HTTP_PROXY HTTPS_PROXY http_proxy https_proxy ALL_PROXY all_proxy
export NO_PROXY=127.0.0.1,localhost
export no_proxy="$NO_PROXY"
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"

# ROS/catkin setup hooks legitimately probe unset variables. Temporarily turn
# off nounset so this runner also works from clean environments (for example a
# transient systemd user service), then restore the strict shell setting.
set +u
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
source "$WS/devel/setup.bash"
set -u

echo "[INFO] Playing $BAG"
echo "[INFO] skip=$SKIP duration=$DURATION rate=$RATE wheel_enabled=$WHEEL_ENABLED wheel_R=$WHEEL_VARIANCE blend=$WHEEL_BLEND gate=$WHEEL_GATE replace_lio_vx=$REPLACE_LIO_VX"

LAUNCH_STATUS=0
roslaunch -p "$ROS_MASTER_PORT" robot_loc wheel_vx_bag_regression.launch \
  input_bag:="$BAG" \
  output_bag:="$OUTPUT_BAG" \
  skip:="$SKIP" \
  duration:="$DURATION" \
  rate:="$RATE" \
  wheel_vx_enabled:="$WHEEL_ENABLED" \
  wheel_primary_vx_variance:="$WHEEL_VARIANCE" \
  wheel_lio_vx_blend:="$WHEEL_BLEND" \
  wheel_lio_vx_gate:="$WHEEL_GATE" \
  wheel_vx_replace_lio_vx:="$REPLACE_LIO_VX" \
  >"${RUN_STEM}.launch.log" 2>&1 || LAUNCH_STATUS=$?

if [[ ! -s "$OUTPUT_BAG" ]]; then
  echo "[ERROR] Result bag was not created (roslaunch status $LAUNCH_STATUS): $OUTPUT_BAG" >&2
  echo "[ERROR] See ${RUN_STEM}.launch.log" >&2
  exit 4
fi

echo "[INFO] Recorded result: $OUTPUT_BAG"
rosbag info "$OUTPUT_BAG" | sed -n '1,80p'
