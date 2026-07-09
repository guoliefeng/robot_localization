#!/usr/bin/env bash
set -euo pipefail

WS=${WS:-/home/guoli/proj/localization_ws}
BASE_DIR=${BASE_DIR:-/home/guoli/data/yangpu/lcp/0706}
DATASETS=${DATASETS:-"202 202-1 203 205 209"}
OUTPUT_NAME=${OUTPUT_NAME:-ins_only_init_check_test.bag}
RATE_ARG=${RATE_ARG:-}

source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"

cleanup() {
  set +e
  if [[ -n "${REC_PID:-}" ]]; then
    kill "$REC_PID" 2>/dev/null
    wait "$REC_PID" 2>/dev/null
  fi
  rosnode kill /ins_only_init_check >/dev/null 2>&1
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill "$LAUNCH_PID" 2>/dev/null
    wait "$LAUNCH_PID" 2>/dev/null
  fi
}

trap cleanup EXIT

for dataset in $DATASETS; do
  DATASET_DIR="$BASE_DIR/$dataset"
  OUT_DIR="$DATASET_DIR/test_out"

  if [[ ! -d "$DATASET_DIR" ]]; then
    echo "[WARN] Skip missing dataset directory: $DATASET_DIR"
    continue
  fi

  mapfile -t BAGS < <(find "$DATASET_DIR" -maxdepth 1 -type f -name '223_7_6_*.bag' | sort -V)
  if [[ ${#BAGS[@]} -eq 0 ]]; then
    echo "[WARN] Skip dataset without 223_7_6_*.bag: $DATASET_DIR"
    continue
  fi

  mkdir -p "$OUT_DIR"
  rm -f "$OUT_DIR/$OUTPUT_NAME" "$OUT_DIR/${OUTPUT_NAME%.bag}.bag.active"

  echo "============================================================"
  echo "[INFO] Dataset: $dataset"
  echo "[INFO] Bags: ${#BAGS[@]}"
  echo "[INFO] Output: $OUT_DIR/$OUTPUT_NAME"

  rosparam set /use_sim_time true

  roslaunch robot_localization ins_only_init_check.launch use_sim_time:=true \
    >"$OUT_DIR/ins_only_init_check.launch.log" 2>&1 &
  LAUNCH_PID=$!

  sleep 2

  rosbag record -O "$OUT_DIR/$OUTPUT_NAME" \
    /clock \
    /localization/ins \
    /localization/final_odom_ins_only \
    /chcnav/devpvt \
    >"$OUT_DIR/ins_only_record.log" 2>&1 &
  REC_PID=$!

  sleep 1

  (
    cd "$DATASET_DIR"
    # shellcheck disable=SC2086
    rosbag play "${BAGS[@]}" --clock $RATE_ARG
  ) >"$OUT_DIR/ins_only_play.log" 2>&1 || true

  sleep 2

  kill "$REC_PID" 2>/dev/null || true
  wait "$REC_PID" 2>/dev/null || true
  unset REC_PID

  rosnode kill /ins_only_init_check >/dev/null 2>&1 || true
  kill "$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
  unset LAUNCH_PID

  if [[ -f "$OUT_DIR/$OUTPUT_NAME" ]]; then
    echo "[INFO] Recorded: $OUT_DIR/$OUTPUT_NAME"
    rosbag info "$OUT_DIR/$OUTPUT_NAME" | sed -n '1,60p'
  else
    echo "[ERROR] Missing output bag: $OUT_DIR/$OUTPUT_NAME"
  fi
done

trap - EXIT
cleanup
