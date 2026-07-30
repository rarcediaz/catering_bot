#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="dry-run"
OUTPUT_DIR=""
ROBOT_AP_INTERFACE="${ROBOT_AP_INTERFACE:-wlan0}"
ROBOT_AP_SSID="${ROBOT_AP_SSID:-IntelliTrolley}"
ROBOT_AP_ADDRESS_CIDR="${ROBOT_AP_ADDRESS_CIDR:-10.42.0.1/24}"
ROBOT_AP_COUNTRY="${ROBOT_AP_COUNTRY:-CA}"
ROBOT_AP_CONNECTION_NAME="${ROBOT_AP_CONNECTION_NAME:-intellitrolley-ap}"
ROBOT_AP_PSK_FILE="${ROBOT_AP_PSK_FILE:-}"
ROBOT_AP_BAND="${ROBOT_AP_BAND:-bg}"
ROBOT_AP_CHANNEL="${ROBOT_AP_CHANNEL:-6}"

usage() {
  cat <<'EOF'
Usage: generate_wifi_ap_config.sh [--dry-run|--detect-manager|--output-dir DIR]

This Phase 2 helper never enables or changes networking. --output-dir writes a
disabled NetworkManager keyfile for later review and manual installation.

Configuration environment variables:
  ROBOT_AP_INTERFACE        default: wlan0
  ROBOT_AP_SSID             default: IntelliTrolley
  ROBOT_AP_ADDRESS_CIDR     default: 10.42.0.1/24
  ROBOT_AP_COUNTRY          default: CA
  ROBOT_AP_CONNECTION_NAME  default: intellitrolley-ap
  ROBOT_AP_PSK_FILE         required only with --output-dir
  ROBOT_AP_BAND             default: bg (2.4 GHz)
  ROBOT_AP_CHANNEL          default: 6
EOF
}

while (( $# > 0 )); do
  case "$1" in
    --dry-run)
      MODE="dry-run"
      ;;
    --detect-manager)
      MODE="detect"
      ;;
    --output-dir)
      if (( $# < 2 )); then
        echo "--output-dir requires a path." >&2
        exit 2
      fi
      MODE="generate"
      OUTPUT_DIR="$2"
      shift
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

detect_manager() {
  if command -v nmcli >/dev/null 2>&1; then
    echo "NetworkManager (nmcli available)"
    return
  fi
  if command -v networkctl >/dev/null 2>&1; then
    echo "systemd-networkd tooling detected; verify which service owns ${ROBOT_AP_INTERFACE}."
    return
  fi
  if command -v dhcpcd >/dev/null 2>&1; then
    echo "dhcpcd tooling detected; this generator does not support legacy dhcpcd AP setup."
    return
  fi
  echo "No supported network manager detected."
  return 1
}

if [[ "${MODE}" == "detect" ]]; then
  detect_manager
  exit
fi

if [[ ! "${ROBOT_AP_INTERFACE}" =~ ^[A-Za-z0-9_.:-]+$ ]]; then
  echo "Invalid ROBOT_AP_INTERFACE: ${ROBOT_AP_INTERFACE}" >&2
  exit 2
fi
if [[ ! "${ROBOT_AP_SSID}" =~ ^[A-Za-z0-9._-]{1,32}$ ]]; then
  echo "ROBOT_AP_SSID must contain 1-32 letters, numbers, '.', '_', or '-'." >&2
  exit 2
fi
if [[ ! "${ROBOT_AP_COUNTRY}" =~ ^[A-Z]{2}$ ]]; then
  echo "ROBOT_AP_COUNTRY must be a two-letter uppercase country code." >&2
  exit 2
fi
if [[ ! "${ROBOT_AP_CONNECTION_NAME}" =~ ^[A-Za-z0-9._-]+$ ]]; then
  echo "Invalid ROBOT_AP_CONNECTION_NAME." >&2
  exit 2
fi
if [[ ! "${ROBOT_AP_BAND}" =~ ^(a|bg)$ ]]; then
  echo "ROBOT_AP_BAND must be a (5 GHz) or bg (2.4 GHz)." >&2
  exit 2
fi
if [[ ! "${ROBOT_AP_CHANNEL}" =~ ^[0-9]+$ ]] \
    || (( ROBOT_AP_CHANNEL < 1 || ROBOT_AP_CHANNEL > 165 )); then
  echo "ROBOT_AP_CHANNEL must be an integer between 1 and 165." >&2
  exit 2
fi
python3 - "${ROBOT_AP_ADDRESS_CIDR}" <<'PY'
import ipaddress
import sys

interface = ipaddress.ip_interface(sys.argv[1])
if not interface.ip.is_private:
    raise SystemExit('ROBOT_AP_ADDRESS_CIDR must use a private address')
if interface.ip == interface.network.network_address:
    raise SystemExit('ROBOT_AP_ADDRESS_CIDR must specify a usable gateway address')
PY

if [[ "${MODE}" == "dry-run" ]]; then
  cat <<EOF
Phase 2 Wi-Fi AP configuration preview (no changes made):
  manager:         NetworkManager (required)
  interface:       ${ROBOT_AP_INTERFACE}
  SSID:            ${ROBOT_AP_SSID}
  Pi gateway/CIDR: ${ROBOT_AP_ADDRESS_CIDR}
  country:         ${ROBOT_AP_COUNTRY}
  connection:      ${ROBOT_AP_CONNECTION_NAME}
  band/channel:    ${ROBOT_AP_BAND}/${ROBOT_AP_CHANNEL}
  DHCP/DNS:        NetworkManager IPv4 shared mode
  IPv6:            disabled for the robot-local profile
  password source: ROBOT_AP_PSK_FILE (never written to Git)

Generate a reviewable keyfile later:
  ROBOT_AP_PSK_FILE=/run/secrets/robot-ap-psk \\
    $0 --output-dir /tmp/intellitrolley-ap

Generation does not install, enable, or activate the profile.
EOF
  exit 0
fi

if [[ -z "${OUTPUT_DIR}" ]]; then
  echo "An output directory is required." >&2
  exit 2
fi
if [[ -z "${ROBOT_AP_PSK_FILE}" || ! -f "${ROBOT_AP_PSK_FILE}" ]]; then
  echo "ROBOT_AP_PSK_FILE must name a readable file for --output-dir." >&2
  exit 2
fi

package_dir="$(realpath -m "${SCRIPT_DIR}/..")"
output_dir_absolute="$(realpath -m "${OUTPUT_DIR}")"
psk_file_absolute="$(realpath -m "${ROBOT_AP_PSK_FILE}")"
if [[ "${output_dir_absolute}" == "${package_dir}" \
    || "${output_dir_absolute}" == "${package_dir}/"* ]]; then
  echo "Refusing to write an AP password anywhere inside the source package." >&2
  exit 2
fi
if [[ "${psk_file_absolute}" == "${package_dir}" \
    || "${psk_file_absolute}" == "${package_dir}/"* ]]; then
  echo "Move ROBOT_AP_PSK_FILE outside the source package before generation." >&2
  exit 2
fi

IFS= read -r ap_psk <"${ROBOT_AP_PSK_FILE}"
if (( ${#ap_psk} < 8 || ${#ap_psk} > 63 )); then
  if [[ ! "${ap_psk}" =~ ^[A-Fa-f0-9]{64}$ ]]; then
    echo "The AP password must be 8-63 characters or exactly 64 hexadecimal characters." >&2
    exit 2
  fi
fi
if LC_ALL=C grep -q '[^ -~]' <<<"${ap_psk}"; then
  echo "The AP password must contain printable ASCII characters only." >&2
  exit 2
fi

mkdir -p "${OUTPUT_DIR}"
config_path="${OUTPUT_DIR}/${ROBOT_AP_CONNECTION_NAME}.nmconnection"
umask 077
{
  printf '%s\n' \
    '[connection]' \
    "id=${ROBOT_AP_CONNECTION_NAME}" \
    'type=wifi' \
    "interface-name=${ROBOT_AP_INTERFACE}" \
    'autoconnect=false' \
    'autoconnect-priority=50' \
    '' \
    '[wifi]' \
    'mode=ap' \
    "ssid=${ROBOT_AP_SSID}" \
    "band=${ROBOT_AP_BAND}" \
    "channel=${ROBOT_AP_CHANNEL}" \
    '' \
    '[wifi-security]' \
    'key-mgmt=wpa-psk' \
    "psk=${ap_psk}" \
    '' \
    '[ipv4]' \
    'method=shared' \
    "address1=${ROBOT_AP_ADDRESS_CIDR}" \
    '' \
    '[ipv6]' \
    'method=disabled'
} >"${config_path}"
chmod 0600 "${config_path}"
unset ap_psk

cat <<EOF
Generated ${config_path} with mode 0600.
No network settings were installed, enabled, or activated.
Set the Pi regulatory country to ${ROBOT_AP_COUNTRY}, review the keyfile, and
follow docs/WIFI_AP_PHASE2.md for the explicit enable and rollback steps.
EOF
