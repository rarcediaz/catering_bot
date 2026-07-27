#!/usr/bin/env bash
set -euo pipefail

ROBOT_WORKSPACE="${ROBOT_WORKSPACE:-/home/zrpi/robot_ws}"
ROBOT_LAUNCH_FILE="${ROBOT_LAUNCH_FILE:-rpi_robot.launch.py}"
ROBOT_LIDAR_DEVICE="${ROBOT_LIDAR_DEVICE:-/dev/ttyUSB0}"
ROBOT_MOTOR_DEVICE="${ROBOT_MOTOR_DEVICE:-/dev/ttyACM0}"
DEVICE_WAIT_LOG_INTERVAL_S="${DEVICE_WAIT_LOG_INTERVAL_S:-10}"
ROBOT_WATCHDOG_ENABLED="${ROBOT_WATCHDOG_ENABLED:-true}"
ROBOT_WATCHDOG_TOPIC_CHECKS="${ROBOT_WATCHDOG_TOPIC_CHECKS:-false}"
ROBOT_WATCHDOG_STARTUP_GRACE_S="${ROBOT_WATCHDOG_STARTUP_GRACE_S:-30}"
ROBOT_WATCHDOG_INTERVAL_S="${ROBOT_WATCHDOG_INTERVAL_S:-5}"
ROBOT_WATCHDOG_FAILURE_LIMIT="${ROBOT_WATCHDOG_FAILURE_LIMIT:-3}"
ROBOT_WATCHDOG_TOPIC_TIMEOUT_S="${ROBOT_WATCHDOG_TOPIC_TIMEOUT_S:-4}"
ROBOT_SCAN_TOPIC="${ROBOT_SCAN_TOPIC:-/scan}"
ROBOT_ODOM_TOPIC="${ROBOT_ODOM_TOPIC:-/diff_cont/odom}"
ROBOT_CLEAN_START="${ROBOT_CLEAN_START:-true}"
ROS_LAUNCH_PID=""

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://${ROBOT_WORKSPACE}/install/my_bot/share/my_bot/config/cyclonedds.xml}"

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

collect_stale_robot_pids() {
  ps -eo pid=,comm=,args= | awk -v workspace="${ROBOT_WORKSPACE}" -v current_pid="$$" '
    {
      pid = $1
      process_name = $2
      command = substr($0, index($0, $3))
      if (pid == current_pid || process_name == "awk" || process_name == "ps") {
        next
      }
      if (
        command ~ /ros2 launch my_bot rpi_(autonomy|robot)\.launch\.py/ ||
        index(command, workspace "/install/my_bot/lib/my_bot/") > 0 ||
        index(command, workspace "/install/ydlidar_ros2_driver/") > 0 ||
        command ~ /\/lib\/controller_manager\/ros2_control_node/ ||
        command ~ /\/lib\/robot_state_publisher\/robot_state_publisher/ ||
        command ~ /\/lib\/laser_filters\/scan_to_scan_filter_chain/ ||
        command ~ /\/lib\/twist_mux\/twist_mux/ ||
        command ~ /\/lib\/nav2_[^/]+\// ||
        (
          command ~ /\/lib\/rclcpp_components\/component_container/ &&
          command ~ /__node:=nav2_container/
        )
      ) {
        print pid
      }
    }
  '
}

collect_device_owner_pids() {
  local device
  for device in "${ROBOT_LIDAR_DEVICE}" "${ROBOT_MOTOR_DEVICE}"; do
    if [[ -e "${device}" ]]; then
      fuser "${device}" 2>/dev/null || true
    fi
  done | tr ' ' '\n' | awk '/^[0-9]+$/' | sort -nu
}

signal_pid_list() {
  local signal_name="$1"
  shift
  if (( $# == 0 )); then
    return
  fi
  kill "-${signal_name}" "$@" 2>/dev/null || true
}

wait_for_pid_list() {
  local attempts="$1"
  shift
  local pid
  local any_alive
  for _ in $(seq 1 "${attempts}"); do
    any_alive=false
    for pid in "$@"; do
      if kill -0 "${pid}" 2>/dev/null; then
        any_alive=true
        break
      fi
    done
    if [[ "${any_alive}" == false ]]; then
      return
    fi
    sleep 0.5
  done
}

clean_stale_robot_stack() {
  if [[ ! "${ROBOT_CLEAN_START}" =~ ^(1|true|yes|on)$ ]]; then
    echo "Robot clean-start sweep is disabled."
    return
  fi
  if ! command -v fuser >/dev/null 2>&1; then
    echo "Robot clean start requires 'fuser' from the psmisc package." >&2
    exit 1
  fi

  local -a stale_pids=()
  local -a owner_pids=()
  mapfile -t stale_pids < <(collect_stale_robot_pids)
  mapfile -t owner_pids < <(collect_device_owner_pids)
  if (( ${#stale_pids[@]} == 0 && ${#owner_pids[@]} == 0 )); then
    echo "Robot clean start: no stale processes found."
    return
  fi

  echo "Robot clean start: stopping stale robot processes: ${stale_pids[*]:-none}"
  if (( ${#owner_pids[@]} > 0 )); then
    echo "Robot clean start: releasing device owners: ${owner_pids[*]}"
  fi
  signal_pid_list INT "${stale_pids[@]}"
  signal_pid_list TERM "${owner_pids[@]}"
  wait_for_pid_list 8 "${stale_pids[@]}" "${owner_pids[@]}"

  mapfile -t stale_pids < <(collect_stale_robot_pids)
  mapfile -t owner_pids < <(collect_device_owner_pids)
  signal_pid_list TERM "${stale_pids[@]}" "${owner_pids[@]}"
  wait_for_pid_list 6 "${stale_pids[@]}" "${owner_pids[@]}"

  mapfile -t stale_pids < <(collect_stale_robot_pids)
  mapfile -t owner_pids < <(collect_device_owner_pids)
  if (( ${#stale_pids[@]} > 0 || ${#owner_pids[@]} > 0 )); then
    echo "Robot clean start: force-stopping unresponsive processes."
    signal_pid_list KILL "${stale_pids[@]}" "${owner_pids[@]}"
    wait_for_pid_list 4 "${stale_pids[@]}" "${owner_pids[@]}"
  fi

  mapfile -t stale_pids < <(collect_stale_robot_pids)
  mapfile -t owner_pids < <(collect_device_owner_pids)
  if (( ${#stale_pids[@]} > 0 || ${#owner_pids[@]} > 0 )); then
    echo "Robot clean start failed; processes remain: ${stale_pids[*]} ${owner_pids[*]}" >&2
    exit 1
  fi
  echo "Robot clean start: stale processes removed and serial devices released."
}

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

clean_stale_robot_stack
wait_for_device "${ROBOT_LIDAR_DEVICE}"
wait_for_device "${ROBOT_MOTOR_DEVICE}"

echo "Starting ${ROBOT_LAUNCH_FILE} from ${ROBOT_WORKSPACE} (ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0})."
setsid ros2 launch my_bot "${ROBOT_LAUNCH_FILE}" use_heartbeat:=false &
ROS_LAUNCH_PID=$!

stop_robot_launch() {
  if [[ -z "${ROS_LAUNCH_PID}" ]] || ! kill -0 "${ROS_LAUNCH_PID}" 2>/dev/null; then
    return
  fi

  kill -INT -- "-${ROS_LAUNCH_PID}" 2>/dev/null || true
  for _ in $(seq 1 20); do
    if ! kill -0 "${ROS_LAUNCH_PID}" 2>/dev/null; then
      break
    fi
    sleep 0.5
  done
  if kill -0 "${ROS_LAUNCH_PID}" 2>/dev/null; then
    kill -TERM -- "-${ROS_LAUNCH_PID}" 2>/dev/null || true
    for _ in $(seq 1 10); do
      if ! kill -0 "${ROS_LAUNCH_PID}" 2>/dev/null; then
        break
      fi
      sleep 0.5
    done
  fi
  if kill -0 "${ROS_LAUNCH_PID}" 2>/dev/null; then
    kill -KILL -- "-${ROS_LAUNCH_PID}" 2>/dev/null || true
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
  echo "Robot device watchdog will begin after ${ROBOT_WATCHDOG_STARTUP_GRACE_S}s."
  if [[ "${ROBOT_WATCHDOG_TOPIC_CHECKS}" =~ ^(1|true|yes|on)$ ]]; then
    echo "Robot topic watchdog checks are enabled."
  else
    echo "Robot topic watchdog checks are disabled."
  fi
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
    if [[ "${ROBOT_WATCHDOG_TOPIC_CHECKS}" =~ ^(1|true|yes|on)$ ]]; then
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
