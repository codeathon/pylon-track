#!/usr/bin/env bash
# Install can0-up.sh + systemd unit, enable, and bring can0 UP.
# Invoked from `make` (ALL) via sudo — lab machines: cmake && make is enough.
set -euo pipefail

SERVICE_SRC=${1:?usage: install-can0-host.sh <service> <can0-up.sh> [sbindir]}
CAN0_UP_SRC=${2:?}
SBINDIR=${3:-/usr/local/sbin}
UNIT_DST=/etc/systemd/system/pylon-track-can0.service

install -d "$SBINDIR"
install -m 755 "$CAN0_UP_SRC" "$SBINDIR/can0-up.sh"
install -m 644 "$SERVICE_SRC" "$UNIT_DST"

systemctl daemon-reload
systemctl enable pylon-track-can0.service

# Bring the iface up now if the adapter is present; otherwise enable is enough
# for the next plug/boot (BindsTo=can0 device).
if ip link show can0 >/dev/null 2>&1; then
	systemctl start pylon-track-can0.service
	"$SBINDIR/can0-up.sh" can0 250000
	echo "install-can0-host: can0 is UP (ip addr show can0)"
else
	echo "install-can0-host: unit enabled; plug in USB-CAN then: systemctl start pylon-track-can0" >&2
fi
