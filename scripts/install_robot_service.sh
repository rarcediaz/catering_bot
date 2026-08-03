#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
DEFAULT_WORKSPACE="$(cd "${PACKAGE_DIR}/../.." && pwd)"
ROBOT_WORKSPACE="${ROBOT_WORKSPACE:-${DEFAULT_WORKSPACE}}"
ROBOT_SERVICE_USER="${ROBOT_SERVICE_USER:-$(id -un)}"
ROBOT_LIDAR_DEVICE="${ROBOT_LIDAR_DEVICE:-/dev/ttyUSB0}"
ROBOT_MOTOR_DEVICE="${ROBOT_MOTOR_DEVICE:-/dev/ttyACM0}"
SERVICE_NAME="my-bot-robot.service"
TEMPLATE_PATH="${PACKAGE_DIR}/systemd/${SERVICE_NAME}.in"
DEFAULTS_TEMPLATE="${PACKAGE_DIR}/systemd/my-bot-robot.default"
SYSTEMD_PATH="/etc/systemd/system/${SERVICE_NAME}"
DEFAULTS_PATH="/etc/default/my-bot-robot"
DRY_RUN=false
NO_START=false

usage() {
  cat <<'EOF'
Usage: install_robot_service.sh [--dry-run] [--no-start]

  --dry-run   Check local prerequisites and print the rendered unit; no sudo or
              system changes are performed.
  --no-start  Install and enable the unit without starting it.
EOF
}

while (( $# > 0 )); do
  case "$1" in
    --dry-run)
      DRY_RUN=true
      ;;
    --no-start)
      NO_START=true
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

source_relaxed() {
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
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

if [[ "${ROBOT_SERVICE_USER}" == "root" ]]; then
  cat >&2 <<'EOF'
Run this installer as the normal robot user, not with sudo.
The installer requests sudo only for systemd and dialout-group changes.
EOF
  exit 1
fi
if [[ "${DRY_RUN}" == false \
    && ! -f /etc/systemd/system/my-bot-network-ready.service ]]; then
  cat >&2 <<'EOF'
Install and enable the IntelliTrolley Wi-Fi stack before the robot service:
  ./src/my_bot/scripts/install_wifi_provisioning_service.sh --enable

The ROS service requires the Wi-Fi startup gate so it cannot publish before a
saved Wi-Fi or the recovery hotspot has an IPv4 address.
EOF
  exit 1
fi
if [[ ! -f "${ROBOT_WORKSPACE}/install/setup.bash" ]]; then
  cat >&2 <<EOF
The robot workspace has not been built:
  ${ROBOT_WORKSPACE}/install/setup.bash

Build the Pi hardware packages first:
  cd ${ROBOT_WORKSPACE}
  colcon build --symlink-install --packages-up-to my_bot
EOF
  exit 1
fi
if [[ ! -f "${TEMPLATE_PATH}" ]]; then
  echo "Systemd template not found: ${TEMPLATE_PATH}" >&2
  exit 1
fi
if [[ ! -f "${DEFAULTS_TEMPLATE}" ]]; then
  echo "Service defaults template not found: ${DEFAULTS_TEMPLATE}" >&2
  exit 1
fi
if ! ROS_SETUP_FILE="$(find_ros_setup)"; then
  echo "No ROS 2 setup file was found. Set ROS_SETUP_FILE or ROS_DISTRO." >&2
  exit 1
fi
source_relaxed "${ROS_SETUP_FILE}"
source_relaxed "${ROBOT_WORKSPACE}/install/setup.bash"

missing_commands=()
for required_command in flock fuser readlink setsid timeout python3 ros2; do
  if ! command -v "${required_command}" >/dev/null 2>&1; then
    missing_commands+=("${required_command}")
  fi
done
if (( ${#missing_commands[@]} > 0 )); then
  cat >&2 <<EOF
The Pi is missing hardware-service commands:
  ${missing_commands[*]}

Install the OS utilities and rerun:
  sudo apt install psmisc util-linux python3
EOF
  exit 1
fi

missing_packages=()
for ros_package in \
  controller_manager \
  diff_drive_controller \
  joint_state_broadcaster \
  robot_state_publisher \
  xacro \
  twist_mux \
  laser_filters \
  ydlidar_ros2_driver \
  diffdrive_arduino \
  rmw_cyclonedds_cpp \
  my_bot; do
  if ! ros2 pkg prefix "${ros_package}" >/dev/null 2>&1; then
    missing_packages+=("${ros_package}")
  fi
done
if (( ${#missing_packages[@]} > 0 )); then
  cat >&2 <<EOF
The Pi is missing required hardware-stack ROS packages:
  ${missing_packages[*]}

Install/build those hardware dependencies and source the overlay. Nav2, AMCL,
SLAM, RViz, and map-server packages are not required by this service.
EOF
  exit 1
fi

if ! ROBOT_SERVICE_HOME="$(getent passwd "${ROBOT_SERVICE_USER}" | cut -d: -f6)"; then
  echo "Could not resolve the home directory for ${ROBOT_SERVICE_USER}." >&2
  exit 1
fi
if [[ -z "${ROBOT_SERVICE_HOME}" || ! -d "${ROBOT_SERVICE_HOME}" ]]; then
  echo "Invalid home directory for ${ROBOT_SERVICE_USER}: ${ROBOT_SERVICE_HOME:-empty}" >&2
  exit 1
fi

tmp_unit="$(mktemp)"
trap 'rm -f "${tmp_unit}"' EXIT

sed \
  -e "s|@ROBOT_USER@|${ROBOT_SERVICE_USER}|g" \
  -e "s|@ROBOT_HOME@|${ROBOT_SERVICE_HOME}|g" \
  -e "s|@ROBOT_WORKSPACE@|${ROBOT_WORKSPACE}|g" \
  -e "s|@ROBOT_PACKAGE_DIR@|${PACKAGE_DIR}|g" \
  "${TEMPLATE_PATH}" >"${tmp_unit}"

if ! grep -q '^Environment=ROBOT_LAUNCH_FILE=rpi_robot.launch.py$' "${tmp_unit}"; then
  echo "Rendered unit does not select rpi_robot.launch.py; refusing installation." >&2
  exit 1
fi
if grep -q 'rpi_autonomy.launch.py' "${tmp_unit}"; then
  echo "Rendered unit unexpectedly references rpi_autonomy.launch.py." >&2
  exit 1
fi

for device_name in ROBOT_LIDAR_DEVICE ROBOT_MOTOR_DEVICE; do
  device_path="${!device_name:-}"
  if [[ -n "${device_path}" && ! -c "${device_path}" ]]; then
    echo "WARNING: ${device_name}=${device_path} is not currently present." >&2
  fi
done
if [[ ! -d /dev/serial/by-id ]]; then
  echo "WARNING: /dev/serial/by-id is unavailable; choose stable device paths when hardware is connected." >&2
fi

if [[ "${DRY_RUN}" == true ]]; then
  echo "Dry run successful. Rendered ${SERVICE_NAME}:"
  sed -n '1,240p' "${tmp_unit}"
  exit 0
fi

echo "Installing ${SERVICE_NAME} for ${ROBOT_SERVICE_USER}..."
sudo usermod -a -G dialout "${ROBOT_SERVICE_USER}"
sudo install -m 0644 "${tmp_unit}" "${SYSTEMD_PATH}"
if [[ ! -e "${DEFAULTS_PATH}" ]]; then
  sudo install -m 0644 "${DEFAULTS_TEMPLATE}" "${DEFAULTS_PATH}"
  echo "Installed editable runtime settings at ${DEFAULTS_PATH}."
else
  echo "Preserved existing runtime settings at ${DEFAULTS_PATH}."
fi
sudo systemctl daemon-reload
sudo systemctl enable "${SERVICE_NAME}"
if [[ "${NO_START}" == false ]]; then
  sudo systemctl restart "${SERVICE_NAME}"
  install_result="installed, enabled, and started"
else
  install_result="installed and enabled (not started)"
fi

cat <<EOF

Pi hardware and safety service ${install_result}.

Status:
  sudo systemctl status ${SERVICE_NAME} --no-pager

Logs:
  sudo journalctl -u ${SERVICE_NAME} -f -o cat

Production launch:
  ros2 launch my_bot rpi_robot.launch.py

The service does not start Nav2, AMCL, SLAM, maps, RViz, Mission Control, or a
phone backend. Its first managed start performs one targeted clean-slate sweep.
The startup gate opens automatically only after filtered lidar is fresh and
the raw command stream has remained free of active motion commands for the
startup quiet period.
EOF
