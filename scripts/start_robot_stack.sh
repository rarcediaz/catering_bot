#!/usr/bin/env bash
set -euo pipefail

ROBOT_WORKSPACE="${ROBOT_WORKSPACE:-/home/zrpi/robot_ws}"
ROBOT_LAUNCH_FILE="${ROBOT_LAUNCH_FILE:-rpi_robot.launch.py}"
ROBOT_LIDAR_DEVICE="${ROBOT_LIDAR_DEVICE:-/dev/ttyUSB0}"
ROBOT_MOTOR_DEVICE="${ROBOT_MOTOR_DEVICE:-/dev/ttyACM0}"
DEVICE_WAIT_LOG_INTERVAL_S="${DEVICE_WAIT_LOG_INTERVAL_S:-10}"
ROBOT_WATCHDOG_ENABLED="${ROBOT_WATCHDOG_ENABLED:-true}"
ROBOT_WATCHDOG_STARTUP_GRACE_S="${ROBOT_WATCHDOG_STARTUP_GRACE_S:-30}"
ROBOT_WATCHDOG_INTERVAL_S="${ROBOT_WATCHDOG_INTERVAL_S:-5}"
ROBOT_WATCHDOG_FAILURE_LIMIT="${ROBOT_WATCHDOG_FAILURE_LIMIT:-3}"
ROBOT_WATCHDOG_TOPIC_TIMEOUT_S="${ROBOT_WATCHDOG_TOPIC_TIMEOUT_S:-4}"
ROBOT_SCAN_TOPIC="${ROBOT_SCAN_TOPIC:-/scan}"
ROBOT_ODOM_TOPIC="${ROBOT_ODOM_TOPIC:-/diff_cont/odom}"
ROS_LAUNCH_PID=""

find_ros_setup() {
  if [[ -n "${ROS_SETUP_FILE:-}" && -f "${ROS_SETUP_FILE}" ]]; then
    printf '%s\n' "${ROS_SETUP_FILE}"
    return
  fi
  if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    printf '%s\n' "/opt/ros/${ROS_DISTRO}/setup.bash"
    return
  fi
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    printf '%s\n' /opt/ros/humble/setup.bash
    return
  fi
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    printf '%s\n' /opt/ros/jazzy/setup.bash
    return
  fi
  return 1
}

source_relaxed() {
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

if ! ROS_SETUP_FILE="$(find_ros_setup)"; then
  echo "No ROS 2 setup file was found. Set ROS_SETUP_FILE or ROS_DISTRO." >&2
  exit 1
fi

if [[ ! -f "${ROBOT_WORKSPACE}/install/setup.bash" ]]; then
  echo "Robot workspace is not built: ${ROBOT_WORKSPACE}/install/setup.bash is missing." >&2
  exit 1
fi

source_relaxed "${ROS_SETUP_FILE}"
source_relaxed "${ROBOT_WORKSPACE}/install/setup.bash"

for numeric_value in \
  "${DEVICE_WAIT_LOG_INTERVAL_S}" \
  "${ROBOT_WATCHDOG_STARTUP_GRACE_S}" \
  "${ROBOT_WATCHDOG_INTERVAL_S}" \
  "${ROBOT_WATCHDOG_FAILURE_LIMIT}" \
  "${ROBOT_WATCHDOG_TOPIC_TIMEOUT_S}"; do
  if [[ ! "${numeric_value}" =~ ^[1-9][0-9]*$ ]]; then
    echo "Robot service timing values must be positive integers (got '${numeric_value}')." >&2
    exit 1
  fi
done

wait_for_device() {
  local device="$1"
  local waited=0

  until [[ -r "${device}" && -w "${device}" ]]; do
    if (( waited % DEVICE_WAIT_LOG_INTERVAL_S == 0 )); then
      echo "Waiting for robot device ${device} to become readable and writable..."
    fi
    sleep 1
    waited=$((waited + 1))
  done

  echo "Robot device ready: ${device}"
}

wait_for_device "${ROBOT_LIDAR_DEVICE}"
wait_for_device "${ROBOT_MOTOR_DEVICE}"

echo "Starting ${ROBOT_LAUNCH_FILE} from ${ROBOT_WORKSPACE} (ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0})."
ros2 launch my_bot "${ROBOT_LAUNCH_FILE}" use_heartbeat:=false &
ROS_LAUNCH_PID=$!

stop_robot_launch() {
  if [[ -z "${ROS_LAUNCH_PID}" ]] || ! kill -0 "${ROS_LAUNCH_PID}" 2>/dev/null; then
    return
  fi

  kill -INT "${ROS_LAUNCH_PID}" 2>/dev/null || true
  for _ in $(seq 1 20); do
    if ! kill -0 "${ROS_LAUNCH_PID}" 2>/dev/null; then
      break
    fi
    sleep 0.5
  done
  if kill -0 "${ROS_LAUNCH_PID}" 2>/dev/null; then
    kill -TERM "${ROS_LAUNCH_PID}" 2>/dev/null || true
  fi
  wait "${ROS_LAUNCH_PID}" 2>/dev/null || true
}

handle_shutdown() {
  trap - INT TERM HUP
  stop_robot_launch
  exit 0
}

trap handle_shutdown INT TERM HUP

if [[ "${ROBOT_WATCHDOG_ENABLED}" =~ ^(1|true|yes|on)$ ]]; then
  echo "Robot watchdog will begin after ${ROBOT_WATCHDOG_STARTUP_GRACE_S}s."
  for _ in $(seq 1 "${ROBOT_WATCHDOG_STARTUP_GRACE_S}"); do
    if ! kill -0 "${ROS_LAUNCH_PID}" 2>/dev/null; then
      wait "${ROS_LAUNCH_PID}" || exit $?
    fi
    sleep 1
  done

  consecutive_failures=0
  while kill -0 "${ROS_LAUNCH_PID}" 2>/dev/null; do
    health_ok=true

    if [[ ! -r "${ROBOT_LIDAR_DEVICE}" || ! -w "${ROBOT_LIDAR_DEVICE}" ]]; then
      echo "Robot watchdog: lidar device is unavailable (${ROBOT_LIDAR_DEVICE})." >&2
      health_ok=false
    fi
    if [[ ! -r "${ROBOT_MOTOR_DEVICE}" || ! -w "${ROBOT_MOTOR_DEVICE}" ]]; then
      echo "Robot watchdog: motor device is unavailable (${ROBOT_MOTOR_DEVICE})." >&2
      health_ok=false
    fi
    if ! timeout "${ROBOT_WATCHDOG_TOPIC_TIMEOUT_S}" \
        ros2 topic echo "${ROBOT_SCAN_TOPIC}" --once --qos-reliability best_effort \
        >/dev/null 2>&1; then
      echo "Robot watchdog: no lidar message on ${ROBOT_SCAN_TOPIC}." >&2
      health_ok=false
    fi
    if ! timeout "${ROBOT_WATCHDOG_TOPIC_TIMEOUT_S}" \
        ros2 topic echo "${ROBOT_ODOM_TOPIC}" --once >/dev/null 2>&1; then
      echo "Robot watchdog: no odometry message on ${ROBOT_ODOM_TOPIC}." >&2
      health_ok=false
    fi

    if [[ "${health_ok}" == true ]]; then
      consecutive_failures=0
    else
      consecutive_failures=$((consecutive_failures + 1))
      echo "Robot watchdog failure ${consecutive_failures}/${ROBOT_WATCHDOG_FAILURE_LIMIT}." >&2
      if (( consecutive_failures >= ROBOT_WATCHDOG_FAILURE_LIMIT )); then
        echo "Robot watchdog is restarting the complete hardware stack." >&2
        stop_robot_launch
        exit 1
      fi
    fi

    sleep "${ROBOT_WATCHDOG_INTERVAL_S}"
  done
fi

set +e
wait "${ROS_LAUNCH_PID}"
launch_status=$?
set -e
exit "${launch_status}"
