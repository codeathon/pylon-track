#!/usr/bin/env python3
"""Minimal CANSimple velocity smoke test for lab node_id 62.

If this spins the motor for ~2s but arena_experiment setup does not, the bug is
in our C++ client. If this also does not spin, fix control_mode/vel_limit in the
ODrive GUI (USB) first.
"""

from __future__ import annotations

import argparse
import struct
import time

import can


def main() -> int:
	p = argparse.ArgumentParser()
	p.add_argument("--iface", default="can0")
	p.add_argument("--node-id", type=int, default=62)
	p.add_argument("--vel", type=float, default=1.0, help="turns/s")
	p.add_argument("--seconds", type=float, default=2.0)
	args = p.parse_args()

	node = args.node_id
	bus = can.interface.Bus(args.iface, interface="socketcan")

	def send(cmd: int, data: bytes) -> None:
		bus.send(can.Message(
			arbitration_id=(node << 5) | cmd,
			data=data,
			is_extended_id=False,
		))

	# Drain stale RX
	while bus.recv(timeout=0) is not None:
		pass

	send(0x18, struct.pack("<B", 0))  # Clear_Errors
	send(0x07, struct.pack("<I", 1))  # IDLE
	time.sleep(0.2)
	send(0x0B, struct.pack("<II", 2, 1))  # VELOCITY + PASSTHROUGH
	send(0x0F, struct.pack("<ff", 10.0, 40.0))  # vel/current limits
	send(0x07, struct.pack("<I", 8))  # CLOSED_LOOP
	time.sleep(0.3)

	print(f"Spinning node {node} at {args.vel} turns/s for {args.seconds}s...")
	t_end = time.monotonic() + args.seconds
	while time.monotonic() < t_end:
		send(0x0D, struct.pack("<ff", args.vel, 0.0))
		time.sleep(0.1)
	send(0x0D, struct.pack("<ff", 0.0, 0.0))
	send(0x07, struct.pack("<I", 1))  # IDLE
	print("Done.")
	return 0


if __name__ == "__main__":
	raise SystemExit(main())
