#!/usr/bin/env python3
"""
One-time ODrive S1 USB bring-up for the prey chain drive.

Configures closed-loop velocity mode, measures chain_mm_per_motor_turn and
chain_direction_sign, then writes results into arena_experiment.json.

Runtime experiments use SocketCAN from C++ (PreyMotor); this script is USB-only
setup via the official odrive Python package.

Requires:
  pip install odrive

Usage (from repo root):
  python src/motor/calibrate_odrive.py --config config/arena_experiment.json
  python src/motor/calibrate_odrive.py --config config/arena_experiment.json --serial <SN>
  python src/motor/calibrate_odrive.py --dry-run   # print CAN checklist only
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
DEFAULT_CONFIG = REPO_ROOT / "config" / "arena_experiment.json"


def load_config(path: Path) -> dict:
	with open(path, "r", encoding="utf-8") as f:
		return json.load(f)


def save_config(path: Path, cfg: dict) -> None:
	with open(path, "w", encoding="utf-8") as f:
		json.dump(cfg, f, indent="\t")
		f.write("\n")
	print(f"Wrote motor calibration → {path}")


def print_can_checklist(cfg: dict) -> None:
	motor = cfg.get("motor", {})
	print("\n--- CAN runtime checklist (after USB setup) ---")
	print("1. In odrivetool or ODrive Web GUI, enable CAN and set node_id =",
		motor.get("node_id", 0))
	print("2. Bring up SocketCAN on the experiment PC, e.g.:")
	print("   sudo ip link set can0 up type can bitrate 250000")
	print("3. Verify heartbeat:")
	print(f"   candump can0 | grep '{motor.get('node_id', 0):03X}'")
	print("4. arena_experiment uses can_interface =", motor.get("can_interface", "can0"))
	print("5. Run: ./build/bin/arena_experiment run --config config/arena_experiment.json")


def connect_odrive(serial_number):
	try:
		import odrive
		from odrive.enums import *
	except ImportError:
		print("ERROR: pip install odrive", file=sys.stderr)
		sys.exit(1)

	print("Searching for ODrive over USB...")
	if serial_number:
		odrv = odrive.find_any(serial_number=serial_number, timeout=30)
	else:
		odrv = odrive.find_any(timeout=30)
	if odrv is None:
		print("ERROR: No ODrive found", file=sys.stderr)
		sys.exit(1)
	print(f"Connected: {odrv.serial_number}")
	return odrv


def setup_axis_velocity(axis) -> None:
	from odrive.enums import (
		AXIS_STATE_CLOSED_LOOP_CONTROL,
		CONTROL_MODE_VELOCITY_CONTROL,
	)

	axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
	axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
	deadline = time.time() + 10.0
	while axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
		if time.time() > deadline:
			raise RuntimeError("Axis failed to enter closed-loop control")
		time.sleep(0.1)
	print("Axis in closed-loop velocity mode")


def run_velocity_test(axis, turns_per_s: float, duration_s: float) -> float:
	"""Spin at constant velocity; return encoder delta in motor turns."""
	pos_start = axis.encoder.pos_estimate
	axis.controller.input_vel = turns_per_s
	time.sleep(duration_s)
	axis.controller.input_vel = 0.0
	time.sleep(0.2)
	pos_end = axis.encoder.pos_estimate
	return pos_end - pos_start


def prompt_float(prompt: str) -> float:
	while True:
		raw = input(prompt).strip()
		try:
			return float(raw)
		except ValueError:
			print("Enter a number.")


def prompt_direction() -> int:
	while True:
		raw = input(
			"Did the chain move in the prey-flee direction? [y/n]: "
		).strip().lower()
		if raw in ("y", "yes"):
			return 1
		if raw in ("n", "no"):
			return -1
		print("Answer y or n.")


def calibrate_chain(cfg: dict, serial_number, test_turns_s: float,
	test_duration_s: float) -> dict:
	odrv = connect_odrive(serial_number)
	axis = odrv.axis0
	setup_axis_velocity(axis)

	motor = cfg.setdefault("motor", {})
	sign = int(motor.get("chain_direction_sign", 1))
	if sign not in (-1, 1):
		sign = 1

	print(f"\nTest spin: {test_turns_s * sign:.2f} turns/s for {test_duration_s:.1f}s")
	print("Keep hands clear of the chain.")
	input("Press Enter to start test spin...")

	delta_turns = run_velocity_test(axis, test_turns_s * sign, test_duration_s)
	print(f"Encoder delta: {delta_turns:.4f} motor turns")

	measured_mm = prompt_float(
		"Measure chain travel during the test (mm, absolute value): "
	)
	if abs(delta_turns) < 1e-4:
		print("ERROR: encoder did not move — check motor wiring and calibration",
			file=sys.stderr)
		sys.exit(1)

	chain_mm_per_turn = abs(measured_mm / delta_turns)
	motor["chain_mm_per_motor_turn"] = round(chain_mm_per_turn, 2)

	# Derive pulley radius for reference (chain_mm = 2*pi*r*1000 per turn).
	motor["pulley_radius_m"] = round(
		chain_mm_per_turn / (2.0 * math.pi * 1000.0), 5)

	print(f"chain_mm_per_motor_turn = {motor['chain_mm_per_motor_turn']}")
	print(f"pulley_radius_m (derived) = {motor['pulley_radius_m']}")

	# Direction check with a short jog in the configured sign.
	print("\nShort direction jog (0.5 s)...")
	run_velocity_test(axis, 0.5 * sign, 0.5)
	new_sign = prompt_direction()
	motor["chain_direction_sign"] = new_sign

	axis.controller.input_vel = 0.0
	print("Calibration complete.")
	return motor


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(
		description="ODrive S1 USB chain calibration for arena_experiment.json")
	parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG,
		help="Path to arena_experiment.json")
	parser.add_argument("--serial", default=None,
		help="ODrive serial number (optional)")
	parser.add_argument("--test-turns-s", type=float, default=1.0,
		help="Test velocity in motor turns/s")
	parser.add_argument("--test-duration-s", type=float, default=2.0,
		help="Test spin duration in seconds")
	parser.add_argument("--dry-run", action="store_true",
		help="Print CAN checklist only; do not connect to ODrive")
	return parser.parse_args()


def main() -> int:
	args = parse_args()
	cfg_path = args.config.resolve()
	if not cfg_path.is_file():
		print(f"ERROR: config not found: {cfg_path}", file=sys.stderr)
		return 1

	cfg = load_config(cfg_path)
	if args.dry_run:
		print_can_checklist(cfg)
		return 0

	motor = calibrate_chain(cfg, args.serial, args.test_turns_s, args.test_duration_s)
	cfg["motor"] = motor
	save_config(cfg_path, cfg)
	print_can_checklist(cfg)
	return 0


if __name__ == "__main__":
	sys.exit(main())
