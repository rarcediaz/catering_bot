#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
DEFAULT_WORKSPACE="$(cd "${PACKAGE_DIR}/../.." && pwd)"
ROBOT_WORKSPACE="${ROBOT_WORKSPACE:-${DEFAULT_WORKSPACE}}"
ROBOT_SERVICE_USER="${ROBOT_SERVICE_USER:-$(id -un)}"
SERVICE_NAME="my-bot-robot.service"
TEMPLATE_PATH="${PACKAGE_DIR}/systemd/${SERVICE_NAME}.in"
SYSTEMD_PATH="/etc/systemd/system/${SERVICE_NAME}"

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
The installer requests sudo only for the systemd installation steps.
EOF
  exit 1
fi

if [[ ! -f "${ROBOT_WORKSPACE}/install/setup.bash" ]]; then
  cat >&2 <<EOF
The robot workspace has not been built:
  ${ROBOT_WORKSPACE}/install/setup.bash

Build it first:
  cd ${ROBOT_WORKSPACE}
  colcon build --symlink-install
EOF
  exit 1
fi

if [[ ! -f "${TEMPLATE_PATH}" ]]; then
  echo "Systemd template not found: ${TEMPLATE_PATH}" >&2
  exit 1
fi

if ! ROS_SETUP_FILE="$(find_ros_setup)"; then
  echo "No ROS 2 setup file was found. Set ROS_SETUP_FILE or ROS_DISTRO." >&2
  exit 1
fi
source_relaxed "${ROS_SETUP_FILE}"
source_relaxed "${ROBOT_WORKSPACE}/install/setup.bash"

missing_commands=()
for required_command in fuser setsid; do
  if ! command -v "${required_command}" >/dev/null 2>&1; then
    missing_commands+=("${required_command}")
  fi
done
if (( ${#missing_commands[@]} > 0 )); then
  cat >&2 <<EOF
The Pi is missing service process-control commands:
  ${missing_commands[*]}

Install them and rerun this installer:
  sudo apt install psmisc util-linux
EOF
  exit 1
fi

missing_packages=()
for ros_package in \
  nav2_bringup \
  nav2_map_server \
  nav2_lifecycle_manager \
  twist_mux \
  laser_filters \
  rmw_cyclonedds_cpp; do
  if ! ros2 pkg prefix "${ros_package}" >/dev/null 2>&1; then
    missing_packages+=("${ros_package}")
  fi
done
if (( ${#missing_packages[@]} > 0 )); then
  cat >&2 <<EOF
The Pi is missing required autonomy packages:
  ${missing_packages[*]}

Install workspace dependencies, rebuild, and rerun this installer:
  cd ${ROBOT_WORKSPACE}
  rosdep install --from-paths src --ignore-src -r -y
  colcon build --symlink-install
EOF
  exit 1
fi

tmp_unit="$(mktemp)"
trap 'rm -f "${tmp_unit}"' EXIT

sed \
  -e "s|@ROBOT_USER@|${ROBOT_SERVICE_USER}|g" \
  -e "s|@ROBOT_WORKSPACE@|${ROBOT_WORKSPACE}|g" \
  -e "s|@ROBOT_PACKAGE_DIR@|${PACKAGE_DIR}|g" \
  "${TEMPLATE_PATH}" >"${tmp_unit}"

echo "Installing ${SERVICE_NAME} for ${ROBOT_SERVICE_USER}..."
sudo usermod -a -G dialout "${ROBOT_SERVICE_USER}"
sudo install -m 0644 "${tmp_unit}" "${SYSTEMD_PATH}"
sudo systemctl daemon-reload
sudo systemctl enable "${SERVICE_NAME}"
sudo systemctl restart "${SERVICE_NAME}"

cat <<EOF

Autonomous robot service installed and started.

Status:
  sudo systemctl status ${SERVICE_NAME} --no-pager

Live developer logs:
  sudo journalctl -u ${SERVICE_NAME} -f -o cat

The service will start hardware, safety, Nav2, AMCL, and maps automatically on future boots.
EOF
