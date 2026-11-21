#!/usr/bin/env bash
# Run on the robot's Ubuntu terminal. No-argument mode only inspects the system.
set -euo pipefail

usage() {
  cat <<'HELP'
Usage: bash net.sh [options]

No options: show current network interfaces and inspect the IMU serial port.
This default does not run sudo, load drivers, or change network/serial settings.

  --serial /dev/DEVICE   IMU serial port (default: SERIAL_PORT or /dev/ttyUSB0).
  --load-module NAME    Explicitly load ch341 OR cp210x using sudo modprobe.
                        May be repeated when the attached USB chip is unknown.
  --grant-serial        Grant a+rw on this serial device only (temporary until
                        unplug/reboot). Prefer adding your user to dialout.
  --interface IFACE     LiDAR Ethernet interface; requires --address.
  --address IPV4/CIDR   Add this static IPv4 address; requires --interface.
                        Existing addresses are kept. Repeating the same address
                        is safe. No routes, gateways, or DNS settings are changed.
  -h, --help            Show this help.

Examples (replace interface, address, and serial path for YOUR robot):
  bash net.sh
  bash net.sh --serial /dev/ttyUSB0 --load-module ch341
  bash net.sh --serial /dev/ttyUSB0 --grant-serial
  bash net.sh --interface eth1 --address 192.168.1.102/24

Network changes require explicit options: use the dedicated LiDAR interface.
Do not select the interface carrying your current ToDesk connection.
Persistent serial access: sudo usermod -aG dialout "$USER", then log out/in.
HELP
}

die() { printf 'ERROR: %s\n' "$*" >&2; exit 2; }
need_value() {
  [[ $# -ge 2 && -n "$2" && "$2" != -* ]] || die "Missing value for $1 (see --help)."
}

serial_port="${SERIAL_PORT:-/dev/ttyUSB0}"
network_iface=''
network_address=''
grant_serial=false
modules=()
while (($#)); do
  case "$1" in
    --serial) need_value "$@"; serial_port="$2"; shift 2 ;;
    --interface) need_value "$@"; network_iface="$2"; shift 2 ;;
    --address) need_value "$@"; network_address="$2"; shift 2 ;;
    --load-module)
      need_value "$@"
      case "$2" in ch341|cp210x) modules+=("$2") ;; *) die 'Module must be ch341 or cp210x.' ;; esac
      shift 2 ;;
    --grant-serial) grant_serial=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) die "Unknown option: $1 (see --help)." ;;
  esac
done

# Validate all requested values before performing any privileged operations.
[[ "$serial_port" == /dev/* && "$serial_port" != *$'\n'* ]] || die '--serial must be an absolute /dev/... path.'
if [[ -n "$network_iface" || -n "$network_address" ]]; then
  [[ -n "$network_iface" && -n "$network_address" ]] || die 'Use --interface and --address together.'
  command -v ip >/dev/null || die 'The ip command is missing (install iproute2).'
  [[ "$network_iface" != -* && "$network_iface" != *[[:space:]]* ]] || die 'Invalid interface name.'
  ip link show dev "$network_iface" >/dev/null 2>&1 || die "Interface does not exist: $network_iface"
  [[ "$network_address" =~ ^([0-9]{1,3})\.([0-9]{1,3})\.([0-9]{1,3})\.([0-9]{1,3})/([0-9]{1,2})$ ]] || die 'Address must be IPv4/CIDR, for example 192.168.1.102/24.'
  for octet in "${BASH_REMATCH[@]:1:4}"; do
    ((10#$octet <= 255)) || die 'IPv4 octets must be between 0 and 255.'
  done
  ((10#${BASH_REMATCH[5]} <= 32)) || die 'IPv4 prefix must be between 0 and 32.'
fi
if $grant_serial; then
  [[ -c "$serial_port" ]] || die "Serial device is missing: $serial_port. Connect the IMU/load its driver, then retry."
fi
if (("${#modules[@]}")); then
  command -v modprobe >/dev/null || die 'The modprobe command is missing (install kmod).'
fi

run_privileged() {
  if ((EUID == 0)); then "$@"; else sudo -- "$@"; fi
}

printf '\nCurrent network interfaces and addresses:\n'
if command -v ip >/dev/null; then
  ip -brief address
else
  printf 'ip is unavailable; install iproute2 to inspect/configure networking.\n'
fi

for module in "${modules[@]}"; do
  if [[ -d "/sys/module/$module" ]]; then
    printf 'USB serial module already loaded: %s\n' "$module"
  else
    printf 'Loading USB serial module for this running kernel: %s\n' "$module"
    run_privileged modprobe "$module"
  fi
done

if [[ -n "$network_iface" ]]; then
  # Add, never flush/replace. Do not change default routes used by ToDesk.
  run_privileged ip link set dev "$network_iface" up
  if ip -o -4 addr show dev "$network_iface" | awk '{print $4}' | grep -Fqx -- "$network_address"; then
    printf 'Address already present: %s on %s\n' "$network_address" "$network_iface"
  else
    run_privileged ip address add "$network_address" dev "$network_iface"
  fi
fi

printf '\nIMU serial port: %s\n' "$serial_port"
if [[ -c "$serial_port" ]]; then
  ls -l -- "$serial_port"
  if $grant_serial; then
    run_privileged chmod a+rw -- "$serial_port"
  fi
  if [[ -r "$serial_port" && -w "$serial_port" ]]; then
    printf 'Current user has read/write access.\n'
  else
    printf 'No read/write access. Add your user to dialout and log out/in,\n'
    printf 'or rerun with --grant-serial for temporary access to this port.\n'
  fi
else
  printf 'Device not found. Check USB cable and run: ls /dev/ttyUSB* /dev/ttyACM*\n'
  printf 'Load the matching chip driver explicitly if needed (see --help).\n'
fi
printf '\nInspection/setup finished. Run --help for explicit setup options.\n'
