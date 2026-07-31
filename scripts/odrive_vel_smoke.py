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
	p.add_argument(
		"--calibrate",
		action="store_true",
		help="Run FULL_CALIBRATION_SEQUENCE before the spin (motor will move)",
	)
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

	# ODrive autobaud: silent until host traffic; beacon then wait for heartbeat.
	print("Autobaud beacon...")
	t_end = time.monotonic() + 3.0
	got_hb = False
	while time.monotonic() < t_end:
		send(0x00, b"")
		msg = bus.recv(timeout=0.05)
		if msg is not None and msg.arbitration_id == ((node << 5) | 0x01):
			got_hb = True
			break
	if not got_hb:
		print("No heartbeat after beacon — check ODrive power/CAN wiring")
		return 1

	send(0x18, struct.pack("<B", 0))  # Clear_Errors
	send(0x07, struct.pack("<I", 1))  # IDLE
	time.sleep(0.2)

	if args.calibrate:
		print("Full calibration (state=3)... keep clear of the motor/chain")
		send(0x07, struct.pack("<I", 3))
		t_end = time.monotonic() + 60.0
		saw_busy = False
		while time.monotonic() < t_end:
			msg = bus.recv(timeout=0.2)
			if msg is None or msg.arbitration_id != ((node << 5) | 0x01):
				continue
			_err, state, result, _done = struct.unpack("<IBBB", bytes(msg.data[:7]))
			if state != 1:
				saw_busy = True
			if saw_busy and state == 1:
				print(f"Calibration done (procedure_result={result}, err={_err})")
				if result != 0 or _err != 0:
					return 1
				break
		else:
			print("Calibration timed out")
			return 1

	send(0x0B, struct.pack("<II", 2, 1))  # VELOCITY + PASSTHROUGH
	send(0x0F, struct.pack("<ff", 10.0, 40.0))  # vel/current limits
	send(0x07, struct.pack("<I", 8))  # CLOSED_LOOP
	time.sleep(0.3)

	print(f"Spinning node {node} at {args.vel} turns/s for {args.seconds}s...")
	enc_id = (node << 5) | 0x09
	pos0 = None
	pos_last = None
	last_vel = 0.0
	n = 0
	t_end = time.monotonic() + args.seconds
	while time.monotonic() < t_end:
		send(0x0D, struct.pack("<ff", args.vel, 0.0))
		n += 1
		# Drain RX; keep latest encoder estimates for a motion check.
		while True:
			msg = bus.recv(timeout=0)
			if msg is None:
				break
			if msg.arbitration_id == enc_id and len(msg.data) >= 8:
				pos_last, last_vel = struct.unpack("<ff", bytes(msg.data[:8]))
				if pos0 is None:
					pos0 = pos_last
		if n == 1 or n % 10 == 0:
			delta = 0.0 if pos0 is None or pos_last is None else (pos_last - pos0)
			print(f"  cmd#{n} sample_vel={last_vel:.4f} delta_turns={delta:.4f}")
		time.sleep(0.05)
	send(0x0D, struct.pack("<ff", 0.0, 0.0))
	send(0x07, struct.pack("<I", 1))  # IDLE
	t_drain = time.monotonic() + 0.3
	while time.monotonic() < t_drain:
		msg = bus.recv(timeout=0.05)
		if msg is not None and msg.arbitration_id == enc_id and len(msg.data) >= 8:
			pos_last, last_vel = struct.unpack("<ff", bytes(msg.data[:8]))
	delta = 0.0 if pos0 is None or pos_last is None else (pos_last - pos0)
	print(f"Done. encoder_delta={delta:.4f} turns  last_vel={last_vel:.4f}")
	if abs(delta) < 0.1:
		print("FAIL: motor did not spin — fix vel_limit/control_mode/calibration in GUI")
		return 1
	print("OK: motor moved")
	return 0


if __name__ == "__main__":
	raise SystemExit(main())
