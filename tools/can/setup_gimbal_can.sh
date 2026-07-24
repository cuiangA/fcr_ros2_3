#!/usr/bin/env bash

# Detect the USB-CAN adapter used by the gimbal and configure SocketCAN.
# Interface numbering (can0/can1) is intentionally not assumed.

set -euo pipefail

BITRATE="${GIMBAL_CAN_BITRATE:-1000000}"
RESTART_MS="${GIMBAL_CAN_RESTART_MS:-100}"
TX_QUEUE_LEN="${GIMBAL_CAN_TX_QUEUE_LEN:-100}"
REQUESTED_INTERFACE="${GIMBAL_CAN_INTERFACE:-}"

usage() {
  cat <<'EOF'
Usage: sudo ./setup_gimbal_can.sh [--interface canX]

Automatically selects the SocketCAN interface whose kernel driver is gs_usb,
then configures it for the gimbal.

Environment overrides:
  GIMBAL_CAN_INTERFACE       Explicit interface (must use the gs_usb driver)
  GIMBAL_CAN_BITRATE         Default: 1000000
  GIMBAL_CAN_RESTART_MS      Default: 100
  GIMBAL_CAN_TX_QUEUE_LEN    Default: 100
EOF
}

while (($# > 0)); do
  case "$1" in
    --interface)
      [[ $# -ge 2 ]] || { echo "ERROR: --interface requires a value" >&2; exit 2; }
      REQUESTED_INTERFACE="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

for command_name in ip readlink basename modprobe sleep; do
  command -v "$command_name" >/dev/null 2>&1 || {
    echo "ERROR: required command not found: $command_name" >&2
    exit 1
  }
done

if [[ ${EUID} -ne 0 ]]; then
  command -v sudo >/dev/null 2>&1 || {
    echo "ERROR: run this script as root; sudo is not installed" >&2
    exit 1
  }
  declare -a sudo_args=()
  if [[ -n "$REQUESTED_INTERFACE" ]]; then
    sudo_args=(--interface "$REQUESTED_INTERFACE")
  fi
  exec sudo --preserve-env=GIMBAL_CAN_INTERFACE,GIMBAL_CAN_BITRATE,GIMBAL_CAN_RESTART_MS,GIMBAL_CAN_TX_QUEUE_LEN \
    "$0" "${sudo_args[@]}"
fi

modprobe can
modprobe can_raw
modprobe gs_usb

driver_for_interface() {
  local interface="$1"
  local driver_path
  driver_path="$(readlink -f "/sys/class/net/${interface}/device/driver" 2>/dev/null || true)"
  [[ -n "$driver_path" ]] && basename "$driver_path"
}

detect_gs_usb_interfaces() {
  local interface_path interface
  candidates=()
  for interface_path in /sys/class/net/can*; do
    [[ -e "$interface_path" ]] || continue
    interface="$(basename "$interface_path")"
    if [[ "$(driver_for_interface "$interface")" == "gs_usb" ]]; then
      candidates+=("$interface")
    fi
  done
}

declare -a candidates=()

if [[ -n "$REQUESTED_INTERFACE" ]]; then
  [[ -d "/sys/class/net/${REQUESTED_INTERFACE}" ]] || {
    echo "ERROR: interface does not exist: ${REQUESTED_INTERFACE}" >&2
    exit 1
  }
  actual_driver="$(driver_for_interface "$REQUESTED_INTERFACE")"
  [[ "$actual_driver" == "gs_usb" ]] || {
    echo "ERROR: ${REQUESTED_INTERFACE} uses '${actual_driver:-unknown}', not gs_usb" >&2
    exit 1
  }
  candidates=("$REQUESTED_INTERFACE")
else
  detect_gs_usb_interfaces
fi

case "${#candidates[@]}" in
  0)
    echo "ERROR: no gs_usb SocketCAN interface found" >&2
    echo "Check the USB-CAN adapter with: lsusb" >&2
    exit 1
    ;;
  1)
    interface="${candidates[0]}"
    ;;
  *)
    echo "ERROR: multiple gs_usb interfaces found: ${candidates[*]}" >&2
    echo "Select one explicitly with: --interface canX" >&2
    exit 1
    ;;
esac

echo "Detected gimbal USB-CAN: ${interface} (driver=gs_usb)"
echo "Configuring bitrate=${BITRATE}, restart-ms=${RESTART_MS}, txqueuelen=${TX_QUEUE_LEN}"

configured=false
for attempt in 1 2 3; do
  if [[ "$(driver_for_interface "$interface")" != "gs_usb" ]]; then
    echo "WARN: ${interface} disappeared or changed driver before configuration (attempt ${attempt}/3)" >&2
  elif ip link set "$interface" down 2>/dev/null &&
       ip link set "$interface" type can bitrate "$BITRATE" restart-ms "$RESTART_MS" &&
       ip link set "$interface" txqueuelen "$TX_QUEUE_LEN" &&
       ip link set "$interface" up; then
    configured=true
    break
  else
    echo "WARN: failed to configure ${interface}; waiting for USB-CAN re-enumeration (attempt ${attempt}/3)" >&2
  fi

  command -v udevadm >/dev/null 2>&1 && udevadm settle || true
  sleep 1
  if [[ -z "$REQUESTED_INTERFACE" ]]; then
    detect_gs_usb_interfaces
    if [[ ${#candidates[@]} -eq 1 ]]; then
      interface="${candidates[0]}"
      echo "Re-detected gimbal USB-CAN as ${interface}"
    fi
  fi
done

if [[ "$configured" != true ]]; then
  echo "ERROR: gs_usb adapter disappeared or could not be configured" >&2
  echo "Recovery: stop ROS nodes, unplug/replug the USB-CAN adapter, then rerun this script." >&2
  echo "Do not substitute a board mttcan interface unless the gimbal is physically wired to it." >&2
  exit 1
fi

state="$(cat "/sys/class/net/${interface}/operstate")"
[[ "$state" == "up" || "$state" == "unknown" ]] || {
  echo "ERROR: ${interface} did not enter an operational state (state=${state})" >&2
  exit 1
}

ip -details -statistics link show "$interface"
echo
echo "GIMBAL_CAN_INTERFACE=${interface}"
echo "Launch example:"
echo "  ros2 launch bringup_pkg fcr_gimbal_manual_test.launch.py can_interface:=${interface}"
