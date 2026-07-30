#!/usr/bin/env bash
# Install SocketCAN host stack for the lab PC.
# Invoked from `make` (ALL) via sudo — success means can0 is administratively UP.
#
# Bundles: kernel modules, systemd unit (for reboot), and immediate ip-link bring-up.
set -euo pipefail

SERVICE_SRC=${1:?usage: install-can0-host.sh <service> <can0-up.sh> [sbindir]}
CAN0_UP_SRC=${2:?}
SBINDIR=${3:-/usr/local/sbin}
UNIT_DST=/etc/systemd/system/pylon-track-can0.service
MODULES_DST=/etc/modules-load.d/pylon-track-can.conf
IFACE=can0
BITRATE=250000
WAIT_SECS=15

iface_admin_up() {
	ip -o link show "$1" 2>/dev/null | grep -Eq '<[^>]*[,<]UP[,>]'
}

install -d "$SBINDIR"
install -m 755 "$CAN0_UP_SRC" "$SBINDIR/can0-up.sh"
install -m 644 "$SERVICE_SRC" "$UNIT_DST"

# Load drivers on every boot (ODrive USB-CAN / candleLight → gs_usb; PEAK → peak_usb).
cat > "$MODULES_DST" <<'EOF'
# pylon-track: SocketCAN USB adapters (installed by make)
gs_usb
peak_usb
EOF

modprobe gs_usb 2>/dev/null || true
modprobe peak_usb 2>/dev/null || true

echo "install-can0-host: waiting up to ${WAIT_SECS}s for $IFACE..."
for _ in $(seq 1 "$WAIT_SECS"); do
	if ip link show "$IFACE" >/dev/null 2>&1; then
		break
	fi
	sleep 1
done

if ! ip link show "$IFACE" >/dev/null 2>&1; then
	echo "install-can0-host: ERROR — no '$IFACE' after loading gs_usb/peak_usb." >&2
	echo "  Plug in USB-CAN adapter fully, then re-run: make" >&2
	echo "  Check: lsusb | grep 1d50:606f ; dmesg | tail -30" >&2
	exit 1
fi

# Persist across reboot (best-effort — bring-up below does not depend on this).
systemctl daemon-reload
systemctl enable pylon-track-can0.service 2>/dev/null || true
systemctl start pylon-track-can0.service 2>/dev/null || true

# Authoritative step: same as ODrive host guide (must succeed for make to pass).
"$SBINDIR/can0-up.sh" "$IFACE" "$BITRATE"

if ! iface_admin_up "$IFACE"; then
	echo "install-can0-host: ERROR — $IFACE still not UP after can0-up.sh" >&2
	ip addr show "$IFACE" >&2 || true
	exit 1
fi

echo "install-can0-host: OK — $IFACE is UP (ip addr show $IFACE)"
ip -br link show "$IFACE" || ip addr show "$IFACE"
