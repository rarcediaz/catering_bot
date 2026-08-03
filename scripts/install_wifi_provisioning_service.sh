#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SERVICE_NAME="my-bot-wifi-provisioning.service"
NETWORK_SERVICE_NAME="my-bot-network-ready.service"
TEMPLATE_PATH="${PACKAGE_DIR}/systemd/${SERVICE_NAME}.in"
NETWORK_TEMPLATE_PATH="${PACKAGE_DIR}/systemd/${NETWORK_SERVICE_NAME}.in"
DEFAULTS_TEMPLATE="${PACKAGE_DIR}/systemd/my-bot-wifi-provisioning.default"
SYSTEMD_PATH="/etc/systemd/system/${SERVICE_NAME}"
NETWORK_SYSTEMD_PATH="/etc/systemd/system/${NETWORK_SERVICE_NAME}"
DEFAULTS_PATH="/etc/default/my-bot-wifi-provisioning"
DRY_RUN=false
ENABLE_SERVICE=false

usage() {
  cat <<'EOF'
Usage: install_wifi_provisioning_service.sh [--dry-run] [--enable]

  --dry-run  Validate and print the rendered unit without changing the system.
  --enable   Enable and start the Wi-Fi gate and provisioning UI.

Without --enable, the units and defaults are installed but remain disabled and
stopped. This installer never creates the access-point profile; when enabled,
the gate may activate an already-installed profile.
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
  "${NETWORK_TEMPLATE_PATH}" \
  "${DEFAULTS_TEMPLATE}" \
  "${PACKAGE_DIR}/scripts/wifi_startup.py" \
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
tmp_network_unit="$(mktemp)"
trap 'rm -f "${tmp_unit}" "${tmp_network_unit}"' EXIT
sed \
  -e "s|@ROBOT_PACKAGE_DIR@|${PACKAGE_DIR}|g" \
  "${TEMPLATE_PATH}" >"${tmp_unit}"
sed \
  -e "s|@ROBOT_PACKAGE_DIR@|${PACKAGE_DIR}|g" \
  "${NETWORK_TEMPLATE_PATH}" >"${tmp_network_unit}"

if grep -q '@ROBOT_PACKAGE_DIR@' "${tmp_unit}" "${tmp_network_unit}"; then
  echo "Rendered Wi-Fi unit contains an unresolved package path." >&2
  exit 1
fi
if ! grep -q '^Before=.*my-bot-robot.service' "${tmp_network_unit}"; then
  echo "Rendered network gate does not run before the robot service." >&2
  exit 1
fi

if [[ "${DRY_RUN}" == true ]]; then
  echo "Dry run successful. No network or system settings were changed."
  sed -n '1,240p' "${tmp_network_unit}"
  sed -n '1,240p' "${tmp_unit}"
  exit 0
fi

echo "Installing ${SERVICE_NAME}..."
sudo install -m 0644 "${tmp_network_unit}" "${NETWORK_SYSTEMD_PATH}"
sudo install -m 0644 "${tmp_unit}" "${SYSTEMD_PATH}"
if [[ ! -e "${DEFAULTS_PATH}" ]]; then
  sudo install -m 0644 "${DEFAULTS_TEMPLATE}" "${DEFAULTS_PATH}"
  echo "Installed editable settings at ${DEFAULTS_PATH}."
else
  echo "Preserved existing settings at ${DEFAULTS_PATH}."
fi
sudo systemctl daemon-reload

if [[ "${ENABLE_SERVICE}" == true ]]; then
  sudo systemctl enable "${NETWORK_SERVICE_NAME}" "${SERVICE_NAME}"
  sudo systemctl restart "${NETWORK_SERVICE_NAME}"
  sudo systemctl restart "${SERVICE_NAME}"
  action="installed, enabled, and started after Wi-Fi became ready"
else
  sudo systemctl disable --now "${NETWORK_SERVICE_NAME}" >/dev/null 2>&1 || true
  sudo systemctl disable --now "${SERVICE_NAME}" >/dev/null 2>&1 || true
  action="installed with its startup gate but left disabled and stopped"
fi

cat <<EOF

Wi-Fi provisioning service ${action}.

When enabled, the startup gate uses a saved client Wi-Fi first and starts the
IntelliTrolley AP only when no saved client profile connects. Once the AP is
active, open:
  http://10.42.0.1:8090/

Status:
  sudo systemctl status ${NETWORK_SERVICE_NAME} --no-pager
  sudo systemctl status ${SERVICE_NAME} --no-pager

Logs:
  sudo journalctl -u ${SERVICE_NAME} -f -o cat
EOF
