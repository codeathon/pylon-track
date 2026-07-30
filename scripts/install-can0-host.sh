#!/usr/bin/env bash
# Install SocketCAN host stack for the lab PC.
# Invoked from `make` (ALL) via sudo — goal: cmake && make → can0 UP.
#
# Bundles: kernel modules (gs_usb/peak_usb), systemd unit, bitrate bring-up.
# Cannot create can0 without a USB-CAN adapter present on USB.
set -euo pipefail

SERVICE_SRC=${1:?usage: install-can0-host.sh <service> <can0-up.sh> [sbindir]}
CAN0_UP_SRC=${2:?}
SBINDIR=${3:-/usr/local/sbin}
UNIT_DST=/etc/systemd/system/pylon-track-can0.service
MODULES_DST=/etc/modules-load.d/pylon-track-can.conf
IFACE=can0
WAIT_SECS=15

install -d "$SBINDIR"
install -m 755 "$CAN0_UP_SRC" "$SBINDIR/can0-up.sh"
install -m 644 "$SERVICE_SRC" "$UNIT_DST"

# Load drivers on every boot (ODrive USB-CAN / candleLight → gs_usb; PEAK → peak_usb).
cat > "$MODULES_DST" <<'EOF'
# pylon-track: SocketCAN USB adapters (installed by make)
gs_usb
peak_usb
EOF

# Load now so can0 can appear without a reboot.
modprobe gs_usb 2>/dev/null || true
modprobe peak_usb 2>/dev/null || true

echo "install-can0-host: waiting up to ${WAIT_SECS}s for $IFACE..."
for _ in $(seq 1 "$WAIT_SECS"); do
	if ip link show "$IFACE" >/dev/null 2>&1; then
		break
	fi
	sleep 1
done

systemctl daemon-reload
systemctl enable pylon-track-can0.service

if ! ip link show "$IFACE" >/dev/null 2>&1; then
	echo "install-can0-host: ERROR — no '$IFACE' after loading gs_usb/peak_usb." >&2
	echo "  Plug in a USB-CAN adapter (not the ODrive USB config port), then re-run make." >&2
	echo "  Check: lsusb ; dmesg | tail -30" >&2
	exit 1
fi

systemctl start pylon-track-can0.service
"$SBINDIR/can0-up.sh" "$IFACE" 250000
echo "install-can0-host: $IFACE is UP (verify: ip addr show $IFACE)"
