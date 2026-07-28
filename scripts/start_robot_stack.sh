#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_WORKSPACE="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
ROBOT_WORKSPACE="${ROBOT_WORKSPACE:-${DEFAULT_WORKSPACE}}"
ROBOT_LAUNCH_FILE="${ROBOT_LAUNCH_FILE:-rpi_robot.launch.py}"
ROBOT_LIDAR_DEVICE="${ROBOT_LIDAR_DEVICE:-/dev/ttyUSB0}"
ROBOT_MOTOR_DEVICE="${ROBOT_MOTOR_DEVICE:-/dev/ttyACM0}"
ROBOT_DEVICE_WAIT_TIMEOUT_S="${ROBOT_DEVICE_WAIT_TIMEOUT_S:-0}"
ROBOT_DEVICE_WAIT_LOG_INTERVAL_S="${ROBOT_DEVICE_WAIT_LOG_INTERVAL_S:-10}"
ROBOT_WATCHDOG_ENABLED="${ROBOT_WATCHDOG_ENABLED:-true}"
ROBOT_WATCHDOG_STARTUP_GRACE_S="${ROBOT_WATCHDOG_STARTUP_GRACE_S:-20}"
ROBOT_WATCHDOG_INTERVAL_S="${ROBOT_WATCHDOG_INTERVAL_S:-2}"
ROBOT_WATCHDOG_FAILURE_LIMIT="${ROBOT_WATCHDOG_FAILURE_LIMIT:-3}"
ROBOT_CYCLONEDDS_PEERS="${ROBOT_CYCLONEDDS_PEERS:-}"
ROBOT_CYCLONEDDS_INTERFACE="${ROBOT_CYCLONEDDS_INTERFACE:-}"
ROBOT_CYCLONEDDS_ALLOW_MULTICAST="${ROBOT_CYCLONEDDS_ALLOW_MULTICAST:-spdp}"
ROBOT_SERVICE_LOCK_FILE="${ROBOT_SERVICE_LOCK_FILE:-/tmp/my-bot-service-$(id -u).lock}"
ROBOT_INITIAL_CLEAN_START="${ROBOT_INITIAL_CLEAN_START:-once}"
ROBOT_STATE_DIR="${ROBOT_STATE_DIR:-${XDG_STATE_HOME:-${HOME}/.local/state}/my-bot}"
ROBOT_INITIAL_CLEAN_MARKER="${ROBOT_INITIAL_CLEAN_MARKER:-${ROBOT_STATE_DIR}/initial-clean-start-complete}"
ROS_LAUNCH_PID=""

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

is_true() {
  [[ "$1" =~ ^(1|true|yes|on)$ ]]
}

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

for positive_value in \
  "${ROBOT_DEVICE_WAIT_LOG_INTERVAL_S}" \
  "${ROBOT_WATCHDOG_STARTUP_GRACE_S}" \
  "${ROBOT_WATCHDOG_INTERVAL_S}" \
  "${ROBOT_WATCHDOG_FAILURE_LIMIT}"; do
  if [[ ! "${positive_value}" =~ ^[1-9][0-9]*$ ]]; then
    echo "Robot timing values must be positive integers (got '${positive_value}')." >&2
    exit 1
  fi
done
if [[ ! "${ROBOT_DEVICE_WAIT_TIMEOUT_S}" =~ ^[0-9]+$ ]]; then
  echo "ROBOT_DEVICE_WAIT_TIMEOUT_S must be zero or a positive integer." >&2
  exit 1
fi
if [[ ! "${ROBOT_CYCLONEDDS_ALLOW_MULTICAST}" =~ ^(true|false|spdp)$ ]]; then
  echo "ROBOT_CYCLONEDDS_ALLOW_MULTICAST must be true, false, or spdp." >&2
  exit 1
fi
if [[ ! "${ROBOT_INITIAL_CLEAN_START}" =~ ^(once|always|off)$ ]]; then
  echo "ROBOT_INITIAL_CLEAN_START must be once, always, or off." >&2
  exit 1
fi
if [[ "${ROBOT_LAUNCH_FILE}" != "rpi_robot.launch.py" ]]; then
  echo "The production service only supports rpi_robot.launch.py." >&2
  echo "Run legacy or central-compute diagnostic launches manually, never through systemd." >&2
  exit 1
fi

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

for required_command in flock fuser readlink setsid timeout python3 ros2; do
  if ! command -v "${required_command}" >/dev/null 2>&1; then
    echo "Required robot-service command is unavailable: ${required_command}" >&2
    exit 1
  fi
done

# A wrapper lock prevents a second service/manual wrapper from reaching the
# devices. rpi_robot.launch.py retains its independent launch-level lock.
exec 9>"${ROBOT_SERVICE_LOCK_FILE}"
if ! flock -n 9; then
  echo "Another robot service wrapper is active (lock: ${ROBOT_SERVICE_LOCK_FILE})." >&2
  exit 1
fi
printf '%s\n' "$$" 1>&9

collect_legacy_robot_pids() {
  ps -eo pid=,comm=,args= | awk \
    -v workspace="${ROBOT_WORKSPACE}" \
    -v current_pid="$$" \
    -v parent_pid="${PPID}" '
    {
      pid = $1
      process_name = $2
      command = substr($0, index($0, $3))
      if (
        pid == current_pid ||
        pid == parent_pid ||
        process_name == "awk" ||
        process_name == "ps"
      ) {
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
        command ~ /\/lib\/slam_toolbox\// ||
        command ~ /\/lib\/rviz2\/rviz2/ ||
        command ~ /__node:=amcl/ ||
        command ~ /__node:=(map_server|display_map_server)/
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

collect_cleanup_pids() {
  {
    collect_legacy_robot_pids
    collect_device_owner_pids
  } | sort -nu
}

signal_pid_list() {
  local signal_name="$1"
  shift
  if (( $# > 0 )); then
    kill "-${signal_name}" "$@" 2>/dev/null || true
  fi
}

wait_for_pid_list() {
  local attempts="$1"
  shift
  local any_alive
  local pid
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

perform_initial_clean_start() {
  local -a cleanup_pids=()

  if [[ "${ROBOT_INITIAL_CLEAN_START}" == "off" ]]; then
    echo "Initial clean-slate sweep is disabled."
    return
  fi
  if [[ "${ROBOT_INITIAL_CLEAN_START}" == "once" \
      && -f "${ROBOT_INITIAL_CLEAN_MARKER}" ]]; then
    echo "Initial clean-slate sweep was already completed."
    return
  fi

  echo "Performing the initial clean-slate sweep for robot and legacy autonomy processes."
  mapfile -t cleanup_pids < <(collect_cleanup_pids)
  if (( ${#cleanup_pids[@]} > 0 )); then
    echo "Stopping existing robot process/device owner PID(s): ${cleanup_pids[*]}"
    signal_pid_list INT "${cleanup_pids[@]}"
    wait_for_pid_list 10 "${cleanup_pids[@]}"
    mapfile -t cleanup_pids < <(collect_cleanup_pids)
  fi
  if (( ${#cleanup_pids[@]} > 0 )); then
    signal_pid_list TERM "${cleanup_pids[@]}"
    wait_for_pid_list 10 "${cleanup_pids[@]}"
    mapfile -t cleanup_pids < <(collect_cleanup_pids)
  fi
  if (( ${#cleanup_pids[@]} > 0 )); then
    echo "Force-stopping unresponsive robot processes: ${cleanup_pids[*]}" >&2
    signal_pid_list KILL "${cleanup_pids[@]}"
    wait_for_pid_list 6 "${cleanup_pids[@]}"
    mapfile -t cleanup_pids < <(collect_cleanup_pids)
  fi
  if (( ${#cleanup_pids[@]} > 0 )); then
    echo "Initial clean-slate sweep failed; PID(s) remain: ${cleanup_pids[*]}" >&2
    echo "Stop those processes manually, then restart the service." >&2
    exit 1
  fi

  mkdir -p "${ROBOT_STATE_DIR}"
  printf 'completed by PID %s\n' "$$" >"${ROBOT_INITIAL_CLEAN_MARKER}"
  echo "Initial clean-slate sweep complete; marker: ${ROBOT_INITIAL_CLEAN_MARKER}"
}

configure_cyclonedds() {
  local base_config
  local generator
  local generated_config

  if [[ -n "${CYCLONEDDS_URI:-}" ]]; then
    echo "Using caller-supplied CYCLONEDDS_URI=${CYCLONEDDS_URI}."
    return
  fi

  base_config="${ROBOT_WORKSPACE}/install/my_bot/share/my_bot/config/cyclonedds.xml"
  if [[ ! -f "${base_config}" ]]; then
    echo "Cyclone DDS base config is missing: ${base_config}" >&2
    exit 1
  fi

  if [[ -z "${ROBOT_CYCLONEDDS_PEERS}" \
      && -z "${ROBOT_CYCLONEDDS_INTERFACE}" \
      && "${ROBOT_CYCLONEDDS_ALLOW_MULTICAST}" == "spdp" ]]; then
    export CYCLONEDDS_URI="file://${base_config}"
    return
  fi

  generator="${SCRIPT_DIR}/generate_cyclonedds_config.py"
  if [[ ! -f "${generator}" ]]; then
    generator="${ROBOT_WORKSPACE}/install/my_bot/lib/my_bot/generate_cyclonedds_config.py"
  fi
  if [[ ! -f "${generator}" ]]; then
    echo "Cyclone DDS config generator is missing." >&2
    exit 1
  fi

  generated_config="/tmp/my-bot-cyclonedds-$(id -u).xml"
  python3 "${generator}" \
    --base "${base_config}" \
    --output "${generated_config}" \
    --peers "${ROBOT_CYCLONEDDS_PEERS}" \
    --interface "${ROBOT_CYCLONEDDS_INTERFACE}" \
    --allow-multicast "${ROBOT_CYCLONEDDS_ALLOW_MULTICAST}"
  export CYCLONEDDS_URI="file://${generated_config}"
}

warn_unstable_device_name() {
  local label="$1"
  local device="$2"
  if [[ "${device}" != /dev/serial/by-id/* ]]; then
    echo "WARNING: ${label} uses ${device}; configure a /dev/serial/by-id path before production acceptance." >&2
  fi
}

handle_wait_shutdown() {
  trap - INT TERM HUP
  echo "Robot service stopped while waiting for hardware."
  exit 0
}
trap handle_wait_shutdown INT TERM HUP

wait_for_device() {
  local label="$1"
  local device="$2"
  local waited=0

  until [[ -c "${device}" && -r "${device}" && -w "${device}" ]]; do
    if (( waited % ROBOT_DEVICE_WAIT_LOG_INTERVAL_S == 0 )); then
      echo "Waiting for ${label} device ${device} to become readable and writable..."
    fi
    if (( ROBOT_DEVICE_WAIT_TIMEOUT_S > 0 && waited >= ROBOT_DEVICE_WAIT_TIMEOUT_S )); then
      echo "Timed out waiting for ${label} device ${device}." >&2
      echo "Check the configured path, USB connection, udev state, and dialout membership." >&2
      exit 1
    fi
    sleep 1
    waited=$((waited + 1))
  done

  echo "Robot ${label} device ready: ${device}"
}

assert_devices_unowned() {
  local device
  local lidar_target
  local motor_target
  local owners

  lidar_target="$(readlink -f "${ROBOT_LIDAR_DEVICE}")"
  motor_target="$(readlink -f "${ROBOT_MOTOR_DEVICE}")"
  if [[ "${lidar_target}" == "${motor_target}" ]]; then
    echo "Lidar and motor paths resolve to the same device: ${lidar_target}" >&2
    exit 1
  fi

  for device in "${ROBOT_LIDAR_DEVICE}" "${ROBOT_MOTOR_DEVICE}"; do
    owners="$(fuser "${device}" 2>/dev/null || true)"
    if [[ -n "${owners//[[:space:]]/}" ]]; then
      echo "Refusing to start: ${device} is already owned by PID(s): ${owners}." >&2
      echo "Stop the existing hardware stack or serial diagnostic process first." >&2
      exit 1
    fi
  done
}

perform_initial_clean_start
configure_cyclonedds
warn_unstable_device_name lidar "${ROBOT_LIDAR_DEVICE}"
warn_unstable_device_name motor "${ROBOT_MOTOR_DEVICE}"
wait_for_device lidar "${ROBOT_LIDAR_DEVICE}"
wait_for_device motor "${ROBOT_MOTOR_DEVICE}"
assert_devices_unowned

echo "Starting Pi hardware/safety stack from ${ROBOT_WORKSPACE} (ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0})."
setsid ros2 launch my_bot "${ROBOT_LAUNCH_FILE}" \
  lidar_device:="${ROBOT_LIDAR_DEVICE}" \
  motor_device:="${ROBOT_MOTOR_DEVICE}" &
ROS_LAUNCH_PID=$!

publish_zero_velocity() {
  if [[ -z "${ROS_LAUNCH_PID}" ]] || ! kill -0 "${ROS_LAUNCH_PID}" 2>/dev/null; then
    return
  fi
  echo "Publishing a best-effort zero-velocity safety override before shutdown."
  if ! timeout 2 ros2 topic pub --once \
      /cmd_vel_safety geometry_msgs/msg/Twist '{}' >/dev/null 2>&1; then
    echo "Zero-velocity publish was not acknowledged; controller and Arduino timeouts remain active." >&2
  fi
  sleep 0.1
}

stop_robot_launch() {
  if [[ -z "${ROS_LAUNCH_PID}" ]] || ! kill -0 "${ROS_LAUNCH_PID}" 2>/dev/null; then
    return
  fi

  publish_zero_velocity
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
    echo "Robot launch did not stop cleanly; forcing its process group down." >&2
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

exit_for_unexpected_launch_stop() {
  local launch_status
  set +e
  wait "${ROS_LAUNCH_PID}"
  launch_status=$?
  set -e
  if (( launch_status == 0 )); then
    echo "Robot launch stopped unexpectedly; requesting a clean systemd retry." >&2
    launch_status=1
  fi
  exit "${launch_status}"
}

if is_true "${ROBOT_WATCHDOG_ENABLED}"; then
  echo "Local device watchdog starts after ${ROBOT_WATCHDOG_STARTUP_GRACE_S}s."
  for _ in $(seq 1 "${ROBOT_WATCHDOG_STARTUP_GRACE_S}"); do
    if ! kill -0 "${ROS_LAUNCH_PID}" 2>/dev/null; then
      exit_for_unexpected_launch_stop
    fi
    sleep 1
  done

  consecutive_failures=0
  while kill -0 "${ROS_LAUNCH_PID}" 2>/dev/null; do
    devices_ok=true
    if [[ ! -c "${ROBOT_LIDAR_DEVICE}" || ! -r "${ROBOT_LIDAR_DEVICE}" || ! -w "${ROBOT_LIDAR_DEVICE}" ]]; then
      echo "Robot watchdog: lidar device is unavailable (${ROBOT_LIDAR_DEVICE})." >&2
      devices_ok=false
    fi
    if [[ ! -c "${ROBOT_MOTOR_DEVICE}" || ! -r "${ROBOT_MOTOR_DEVICE}" || ! -w "${ROBOT_MOTOR_DEVICE}" ]]; then
      echo "Robot watchdog: motor device is unavailable (${ROBOT_MOTOR_DEVICE})." >&2
      devices_ok=false
    fi

    if [[ "${devices_ok}" == true ]]; then
      consecutive_failures=0
    else
      consecutive_failures=$((consecutive_failures + 1))
      echo "Robot device failure ${consecutive_failures}/${ROBOT_WATCHDOG_FAILURE_LIMIT}." >&2
      if (( consecutive_failures >= ROBOT_WATCHDOG_FAILURE_LIMIT )); then
        echo "Persistent local device loss; stopping the stack for a safe systemd retry." >&2
        stop_robot_launch
        exit 1
      fi
    fi
    sleep "${ROBOT_WATCHDOG_INTERVAL_S}"
  done
fi

exit_for_unexpected_launch_stop
