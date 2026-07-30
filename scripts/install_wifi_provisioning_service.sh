#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SERVICE_NAME="my-bot-wifi-provisioning.service"
TEMPLATE_PATH="${PACKAGE_DIR}/systemd/${SERVICE_NAME}.in"
DEFAULTS_TEMPLATE="${PACKAGE_DIR}/systemd/my-bot-wifi-provisioning.default"
SYSTEMD_PATH="/etc/systemd/system/${SERVICE_NAME}"
DEFAULTS_PATH="/etc/default/my-bot-wifi-provisioning"
DRY_RUN=false
ENABLE_SERVICE=false

usage() {
  cat <<'EOF'
Usage: install_wifi_provisioning_service.sh [--dry-run] [--enable]

  --dry-run  Validate and print the rendered unit without changing the system.
  --enable   Explicitly enable and start the provisioning UI after installation.

Without --enable, the unit and defaults are installed but remain disabled and
stopped. This installer never creates or activates a Wi-Fi access point.
EOF
}

while (( $# > 0 )); do
  case "$1" in
    --dry-run)
      DRY_RUN=true
      ;;
    --enable)
      ENABLE_SERVICE=true
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

if [[ "$(id -u)" -eq 0 ]]; then
  echo "Run this installer as the normal robot user, not with sudo." >&2
  exit 1
fi

for required_path in \
  "${TEMPLATE_PATH}" \
  "${DEFAULTS_TEMPLATE}" \
  "${PACKAGE_DIR}/scripts/wifi_provisioning_server.py" \
  "${PACKAGE_DIR}/wifi_provisioning_ui/index.html" \
  "${PACKAGE_DIR}/wifi_provisioning_ui/app.js" \
  "${PACKAGE_DIR}/wifi_provisioning_ui/styles.css"; do
  if [[ ! -f "${required_path}" ]]; then
    echo "Required provisioning file is missing: ${required_path}" >&2
    exit 1
  fi
done

missing_commands=()
for required_command in nmcli busctl ip python3 systemctl; do
  if ! command -v "${required_command}" >/dev/null 2>&1; then
    missing_commands+=("${required_command}")
  fi
done
if (( ${#missing_commands[@]} > 0 )); then
  cat >&2 <<EOF
The Pi is missing provisioning commands:
  ${missing_commands[*]}

Install NetworkManager and the standard networking tools before continuing.
EOF
  exit 1
fi
if [[ "$(systemctl is-active NetworkManager 2>/dev/null || true)" != "active" ]]; then
  if [[ "${DRY_RUN}" == true ]]; then
    echo "WARNING: NetworkManager is not active here; the rendered-unit check will continue." >&2
  else
    echo "NetworkManager is not active; refusing to install an unusable provisioner." >&2
    exit 1
  fi
fi

tmp_unit="$(mktemp)"
trap 'rm -f "${tmp_unit}"' EXIT
sed \
  -e "s|@ROBOT_PACKAGE_DIR@|${PACKAGE_DIR}|g" \
  "${TEMPLATE_PATH}" >"${tmp_unit}"

if grep -q '@ROBOT_PACKAGE_DIR@' "${tmp_unit}"; then
  echo "Rendered provisioning unit contains an unresolved package path." >&2
  exit 1
fi
if grep -q 'WantedBy=my-bot-robot' "${tmp_unit}"; then
  echo "Provisioning must remain independent from the robot ROS service." >&2
  exit 1
fi

if [[ "${DRY_RUN}" == true ]]; then
  echo "Dry run successful. No network or system settings were changed."
  sed -n '1,240p' "${tmp_unit}"
  exit 0
fi

echo "Installing ${SERVICE_NAME}..."
sudo install -m 0644 "${tmp_unit}" "${SYSTEMD_PATH}"
if [[ ! -e "${DEFAULTS_PATH}" ]]; then
  sudo install -m 0644 "${DEFAULTS_TEMPLATE}" "${DEFAULTS_PATH}"
  echo "Installed editable settings at ${DEFAULTS_PATH}."
else
  echo "Preserved existing settings at ${DEFAULTS_PATH}."
fi
sudo systemctl daemon-reload

if [[ "${ENABLE_SERVICE}" == true ]]; then
  sudo systemctl enable --now "${SERVICE_NAME}"
  action="installed, enabled, and started"
else
  sudo systemctl disable --now "${SERVICE_NAME}" >/dev/null 2>&1 || true
  action="installed but left disabled and stopped"
fi

cat <<EOF

Wi-Fi provisioning service ${action}.

The current Wi-Fi connection and every NetworkManager profile were left
unchanged. Once the IntelliTrolley AP is active, open:
  http://10.42.0.1:8090/

Status:
  sudo systemctl status ${SERVICE_NAME} --no-pager

Logs:
  sudo journalctl -u ${SERVICE_NAME} -f -o cat
EOF
