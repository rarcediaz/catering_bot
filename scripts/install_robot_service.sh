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

tmp_unit="$(mktemp)"
trap 'rm -f "${tmp_unit}"' EXIT

sed \
  -e "s|@ROBOT_USER@|${ROBOT_SERVICE_USER}|g" \
  -e "s|@ROBOT_WORKSPACE@|${ROBOT_WORKSPACE}|g" \
  "${TEMPLATE_PATH}" >"${tmp_unit}"

echo "Installing ${SERVICE_NAME} for ${ROBOT_SERVICE_USER}..."
sudo usermod -a -G dialout "${ROBOT_SERVICE_USER}"
sudo install -m 0644 "${tmp_unit}" "${SYSTEMD_PATH}"
sudo systemctl daemon-reload
sudo systemctl enable "${SERVICE_NAME}"
sudo systemctl restart "${SERVICE_NAME}"

cat <<EOF

Robot service installed and started.

Status:
  sudo systemctl status ${SERVICE_NAME} --no-pager

Live developer logs:
  sudo journalctl -u ${SERVICE_NAME} -f -o cat

The service will start automatically on future boots.
EOF
