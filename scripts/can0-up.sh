#!/usr/bin/env bash
# Bring up a SocketCAN iface at the ODrive-recommended bitrate (host guide).
# Idempotent: already-up / "Device or resource busy" is success if IFF_UP.
# Success check matches `ip addr show` administrative UP (not operstate).
set -euo pipefail

IFACE=${1:-can0}
BITRATE=${2:-250000}

iface_admin_up() {
	# Flags in angle brackets include UP when IFF_UP is set (same as ip addr).
	ip -o link show "$1" 2>/dev/null | grep -Eq '<[^>]*[,<]UP[,>]'
}

if ! ip link show "$IFACE" >/dev/null 2>&1; then
	echo "can0-up: interface '$IFACE' not found — is the USB-CAN adapter plugged in?" >&2
	exit 1
fi

ERR=$(mktemp)
# Restart link so bitrate is applied cleanly after reboot / re-plug.
ip link set "$IFACE" down 2>/dev/null || true
if ! ip link set "$IFACE" up type can bitrate "$BITRATE" 2>"$ERR"; then
	# ODrive docs: busy often means the interface is already up.
	if ! grep -qi 'busy' "$ERR"; then
		cat "$ERR" >&2
		rm -f "$ERR"
		exit 1
	fi
fi
rm -f "$ERR"

if ! iface_admin_up "$IFACE"; then
	echo "can0-up: '$IFACE' is not UP after bring-up (check: ip addr show $IFACE)" >&2
	exit 1
fi

# ODrive autobaud: drive stays silent until it sees host frames (≥10 Hz).
# Best-effort — can-utils may be missing; apps also beacon on PreyMotor::connect.
if command -v cangen >/dev/null 2>&1; then
	cangen "$IFACE" -I 7C0 -L 0 -g 50 -n 40 >/dev/null 2>&1 || true
	echo "can0-up: sent autobaud beacon (cangen)"
elif command -v cansend >/dev/null 2>&1; then
	for _ in $(seq 1 40); do
		cansend "$IFACE" 7C0# || true
		sleep 0.05
	done
	echo "can0-up: sent autobaud beacon (cansend)"
fi

echo "can0-up: $IFACE UP bitrate $BITRATE"
