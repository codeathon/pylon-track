#!/usr/bin/env python3
"""
Calibrate Basler a2A1920-160um + 4mm lens with a ChArUco board, then save
intrinsics (K) and distortion coefficients for undistortion.

Why ChArUco (not a plain checkerboard): your distortion is worst at the frame
edges, so the calibration MUST see corners out there. A full checkerboard has to
be entirely visible, so you can never push it into the extreme corners. ChArUco
detects partial boards, so you can fill the edges/corners — which is where the
distortion model is actually constrained.

Requires: opencv-contrib-python >= 4.7  (CharucoDetector API), pypylon

Usage:
  python calibration.py --make-board    # render charuco_board.png to print
  python calibration.py --capture       # live grab; SPACE to save, q to quit
  python calibration.py --calibrate     # compute K + dist -> calib.npz

ferret_tracker loads calib.npz automatically when placed beside the binary
(build/bin/calib.npz) or via --calib / PYLON_CAMERA_CALIB. Calibrate at the
same width×height as camera_config.json.
"""

import argparse
import glob
import json
import os
import sys
from pathlib import Path

import cv2
import numpy as np

# ---- ChArUco board definition (match your printed board) --------------------
SQUARES_X = 10          # squares across
SQUARES_Y = 7           # squares down
SQUARE_LEN = 0.035      # square side in metres (measure your print!)
MARKER_LEN = 0.026      # aruco marker side in metres
DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
IMG_DIR = "calib_frames"
DEFAULT_CAMERA_CONFIG = "src/camera_config.json"

# Fraction of frame width/height treated as the "edge band" for coverage checks.
EDGE_MARGIN = 0.12
# 3×3 zone grid — all zones should turn green across saved frames before calibrate.
ZONE_GRID = 3


def make_board():
	return cv2.aruco.CharucoBoard(
		(SQUARES_X, SQUARES_Y), SQUARE_LEN, MARKER_LEN, DICTIONARY)


def make_detector():
	"""Detector tuned for wide-angle vignetting and small markers near borders."""
	det_params = cv2.aruco.DetectorParameters()
	# Wider adaptive threshold search helps low-contrast markers in dark edge bands.
	det_params.adaptiveThreshWinSizeMin = 3
	det_params.adaptiveThreshWinSizeMax = 33
	det_params.adaptiveThreshWinSizeStep = 4
	det_params.adaptiveThreshConstant = 5
	# Allow smaller apparent markers when the board is tilted or near the border.
	det_params.minMarkerPerimeterRate = 0.02
	det_params.maxMarkerPerimeterRate = 4.0
	det_params.minCornerDistanceRate = 0.01
	det_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
	return cv2.aruco.CharucoDetector(make_board(), detectorParams=det_params)


board = make_board()
detector = make_detector()


def resolve_camera_config(cli_path: str) -> Path:
	if cli_path:
		return Path(cli_path)
	for candidate in (
		Path(os.environ.get("PYLON_CAMERA_CONFIG", "")),
		Path("build/bin/camera_config.json"),
		Path("src/camera_config.json"),
		Path(DEFAULT_CAMERA_CONFIG),
	):
		if candidate and candidate.is_file():
			return candidate
	return Path(DEFAULT_CAMERA_CONFIG)


def load_camera_config(path: Path) -> dict:
	with path.open() as handle:
		return json.load(handle)


def apply_camera_config(cam, cfg: dict, exposure_scale: float, gain_offset_db: float):
	"""Mirror production AOI/exposure; optional boost brightens vignetted edges."""
	from pypylon import pylon

	if cfg.get("pixel_format", "Mono8") != "Mono8":
		raise SystemExit("camera_config.json must use Mono8 for calibration capture")

	cam.PixelFormat.SetValue("Mono8")
	cam.Width.SetValue(int(cfg["width"]))
	cam.Height.SetValue(int(cfg["height"]))
	cam.OffsetX.SetValue(int(cfg.get("offset_x", 0)))
	cam.OffsetY.SetValue(int(cfg.get("offset_y", 0)))

	exposure_auto = bool(cfg.get("exposure_auto", False))
	cam.ExposureAuto.SetValue("Continuous" if exposure_auto else "Off")
	if not exposure_auto:
		exposure_us = float(cfg["exposure_time_us"]) * exposure_scale
		cam.ExposureTime.SetValue(exposure_us)

	gain_auto = bool(cfg.get("gain_auto", False))
	cam.GainAuto.SetValue("Continuous" if gain_auto else "Off")
	if not gain_auto:
		gain_db = float(cfg["gain_db"]) + gain_offset_db
		cam.Gain.SetValue(gain_db)

	if bool(cfg.get("frame_rate_enable", True)):
		cam.AcquisitionFrameRateEnable.SetValue(True)
		cam.AcquisitionFrameRate.SetValue(float(cfg.get("frame_rate_fps", 200.0)))
	else:
		cam.AcquisitionFrameRateEnable.SetValue(False)

	if cfg.get("trigger_mode", "Off") != "Off":
		raise SystemExit("calibration capture requires trigger_mode Off")

	print(
		f"camera: {cfg['width']}x{cfg['height']} "
		f"offset=({cfg.get('offset_x', 0)},{cfg.get('offset_y', 0)}) "
		f"exposure_scale={exposure_scale:.2f} gain_offset={gain_offset_db:+.1f}dB"
	)


def detect_board(gray: np.ndarray):
	"""Run ChArUco detection; CLAHE helps markers in dark edge bands."""
	# CLAHE is for detection only — saved PNGs stay raw for an honest calibrate step.
	clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
	enhanced = clahe.apply(gray)
	return detector.detectBoard(enhanced)


def zone_index(x: float, y: float, width: int, height: int) -> tuple[int, int]:
	col = min(ZONE_GRID - 1, max(0, int(x / width * ZONE_GRID)))
	row = min(ZONE_GRID - 1, max(0, int(y / height * ZONE_GRID)))
	return row, col


def coverage_from_corners(corners, width: int, height: int) -> dict:
	"""Summarise which zones and edge bands have at least one corner."""
	zones: set[tuple[int, int]] = set()
	edges = {"left": False, "right": False, "top": False, "bottom": False}
	if corners is None or len(corners) == 0:
		return {"zones": zones, "edges": edges}

	margin_x = width * EDGE_MARGIN
	margin_y = height * EDGE_MARGIN
	for pt in corners.reshape(-1, 2):
		x, y = float(pt[0]), float(pt[1])
		zones.add(zone_index(x, y, width, height))
		if x < margin_x:
			edges["left"] = True
		if x > width - margin_x:
			edges["right"] = True
		if y < margin_y:
			edges["top"] = True
		if y > height - margin_y:
			edges["bottom"] = True
	return {"zones": zones, "edges": edges}


def merge_coverage(saved: dict, live: dict) -> dict:
	return {
		"zones": saved["zones"] | live["zones"],
		"edges": {key: saved["edges"][key] or live["edges"][key] for key in saved["edges"]},
	}


def draw_coverage_hud(disp, width: int, height: int, saved_cov: dict, live_cov: dict):
	"""Overlay 3×3 grid: green=saved, yellow=live only, red=uncovered."""
	cell_w = width // ZONE_GRID
	cell_h = height // ZONE_GRID
	for row in range(ZONE_GRID):
		for col in range(ZONE_GRID):
			x0, y0 = col * cell_w, row * cell_h
			zone = (row, col)
			if zone in saved_cov["zones"]:
				color = (0, 180, 0)
			elif zone in live_cov["zones"]:
				color = (0, 220, 220)
			else:
				color = (0, 0, 220)
			cv2.rectangle(disp, (x0, y0), (x0 + cell_w, y0 + cell_h), color, 2)

	margin_x = int(width * EDGE_MARGIN)
	margin_y = int(height * EDGE_MARGIN)
	cv2.rectangle(disp, (0, 0), (width - 1, height - 1), (255, 255, 0), 1)
	cv2.rectangle(
		disp, (margin_x, margin_y), (width - margin_x, height - margin_y), (180, 180, 180), 1
	)

	edge_labels = (
		("left", (10, height // 2)),
		("right", (width - 70, height // 2)),
		("top", (width // 2 - 20, 24)),
		("bottom", (width // 2 - 30, height - 12)),
	)
	for name, (tx, ty) in edge_labels:
		ok = saved_cov["edges"][name]
		label = f"{name}:{'OK' if ok else 'need'}"
		cv2.putText(
			disp, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
			(0, 255, 0) if ok else (0, 0, 255), 2, cv2.LINE_AA,
		)


def coverage_complete(cov: dict) -> bool:
	want_zones = ZONE_GRID * ZONE_GRID
	return len(cov["zones"]) >= want_zones and all(cov["edges"].values())


def print_coverage_report(cov: dict, prefix: str = ""):
	edges = ", ".join(f"{k}={'yes' if v else 'NO'}" for k, v in cov["edges"].items())
	print(
		f"{prefix}zones {len(cov['zones'])}/{ZONE_GRID * ZONE_GRID}, edges: {edges}"
	)


def make_board_png(path="charuco_board.png", px_per_square=240):
	"""Render the board to print. Print FLAT on rigid backing; verify SQUARE_LEN after printing."""
	img = board.generateImage((SQUARES_X * px_per_square, SQUARES_Y * px_per_square))
	cv2.imwrite(path, img)
	print(f"wrote {path} -- print it, mount flat, then MEASURE a square and update SQUARE_LEN")


def capture(camera_config: Path, exposure_scale: float, gain_offset_db: float):
	"""Grab frames from the Basler; live HUD shows edge/zone coverage."""
	from pypylon import pylon

	cfg = load_camera_config(camera_config)
	if not camera_config.is_file():
		raise SystemExit(f"camera config not found: {camera_config}")

	os.makedirs(IMG_DIR, exist_ok=True)
	cam = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
	cam.Open()
	apply_camera_config(cam, cfg, exposure_scale, gain_offset_db)
	cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
	conv = pylon.ImageFormatConverter()
	conv.OutputPixelFormat = pylon.PixelType_Mono8

	n = len(glob.glob(f"{IMG_DIR}/*.png"))
	saved_cov = {"zones": set(), "edges": {k: False for k in ("left", "right", "top", "bottom")}}
	width = int(cfg["width"])
	height = int(cfg["height"])

	print(
		"SPACE = save frame, q = quit.\n"
		"Push the board into each YELLOW corner/edge until all grid cells and edge labels are GREEN.\n"
		"Tip: hold only a corner of the board in the frame — the full sheet does not need to fit.\n"
		f"Using camera config: {camera_config}"
	)

	while cam.IsGrabbing():
		res = cam.RetrieveResult(2000, pylon.TimeoutHandling_ThrowException)
		if not res.GrabSucceeded():
			res.Release()
			continue

		img = conv.Convert(res).GetArray()
		disp = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
		cc, cids, _, _ = detect_board(img)
		if cids is not None and len(cids) > 0:
			cv2.aruco.drawDetectedCornersCharuco(disp, cc, cids)

		live_cov = coverage_from_corners(cc, width, height)
		draw_coverage_hud(disp, width, height, saved_cov, live_cov)

		status = "READY" if coverage_complete(saved_cov) else "cover edges"
		cv2.putText(
			disp, f"saved:{n}  corners:{0 if cids is None else len(cids)}  {status}",
			(20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2,
		)
		cv2.imshow("capture", cv2.resize(disp, None, fx=0.6, fy=0.6))
		key = cv2.waitKey(1) & 0xFF

		if key == ord(" "):
			if cids is None or len(cids) < 6:
				print("  skip save — need >=6 detected corners in this frame")
			elif not all(live_cov["edges"].values()):
				missing = [name for name, ok in live_cov["edges"].items() if not ok]
				print(
					f"  warning: frame lacks edge corners ({', '.join(missing)}) — "
					"saved anyway; rounded undistort corners mean weak edge data"
				)
				cv2.imwrite(f"{IMG_DIR}/frame_{n:03d}.png", img)
				n += 1
				saved_cov = merge_coverage(saved_cov, live_cov)
				print(f"saved frame_{n - 1:03d}.png")
				print_coverage_report(saved_cov, "  coverage: ")
			else:
				cv2.imwrite(f"{IMG_DIR}/frame_{n:03d}.png", img)
				n += 1
				saved_cov = merge_coverage(saved_cov, live_cov)
				print(f"saved frame_{n - 1:03d}.png")
				print_coverage_report(saved_cov, "  coverage: ")
		elif key == ord("q"):
			res.Release()
			break
		res.Release()

	cam.StopGrabbing()
	cam.Close()
	cv2.destroyAllWindows()
	print(f"{n} frames in {IMG_DIR}/")
	print_coverage_report(saved_cov, "final coverage: ")
	if not coverage_complete(saved_cov):
		print(
			"WARNING: edge/zone coverage incomplete — undistortion will look fine in the "
			"centre but rounded/warped near corners. Re-run --capture and fill red zones."
		)


def calibrate(use_rational_model=True):
	"""Compute K + distortion from saved frames; save to calib.npz."""
	files = sorted(glob.glob(f"{IMG_DIR}/*.png"))
	if len(files) < 12:
		sys.exit(f"only {len(files)} frames -- grab ~20-40 covering the whole frame")

	objpoints, imgpoints, img_size = [], [], None
	total_cov = {"zones": set(), "edges": {k: False for k in ("left", "right", "top", "bottom")}}

	for path in files:
		img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
		img_size = img.shape[::-1]  # (w, h)
		cc, cids, _, _ = detect_board(img)
		if cids is None or len(cids) < 6:
			print(f"  skip {os.path.basename(path)} (only {0 if cids is None else len(cids)} corners)")
			continue
		obj_pts, img_pts = board.matchImagePoints(cc, cids)
		if obj_pts is None or len(obj_pts) < 6:
			continue
		objpoints.append(obj_pts)
		imgpoints.append(img_pts)
		frame_cov = coverage_from_corners(cc, img_size[0], img_size[1])
		total_cov = merge_coverage(total_cov, frame_cov)

	print(f"using {len(objpoints)} usable frames")
	print_coverage_report(total_cov, "dataset coverage: ")
	if not coverage_complete(total_cov):
		print(
			"WARNING: saved frames do not cover all edge bands / zones — expect rounded "
			"corners after undistort. Re-capture with the on-screen coverage HUD green."
		)

	flags = cv2.CALIB_RATIONAL_MODEL if use_rational_model else 0
	rms, k_mat, dist, _rvecs, _tvecs = cv2.calibrateCamera(
		objpoints, imgpoints, img_size, None, None, flags=flags
	)

	print(f"\nRMS reprojection error: {rms:.4f} px   (aim for < ~0.3; >0.6 => recapture)")
	print("K =\n", k_mat)
	print("dist =", dist.ravel())
	np.savez("calib.npz", K=k_mat, dist=dist, img_size=img_size, rms=rms)
	print("\nsaved calib.npz")


if __name__ == "__main__":
	ap = argparse.ArgumentParser()
	ap.add_argument("--make-board", action="store_true")
	ap.add_argument("--capture", action="store_true")
	ap.add_argument("--calibrate", action="store_true")
	ap.add_argument("--no-rational", action="store_true", help="use plain 5-coeff model")
	ap.add_argument("--camera-config", default="", help="AOI/exposure JSON (default: src/camera_config.json)")
	ap.add_argument(
		"--exposure-scale", type=float, default=2.0,
		help="multiply exposure_time_us during capture to brighten vignetted edges (default 2.0)",
	)
	ap.add_argument(
		"--gain-offset-db", type=float, default=3.0,
		help="add gain (dB) during capture for edge marker visibility (default +3)",
	)
	args = ap.parse_args()
	config_path = resolve_camera_config(args.camera_config)

	if args.make_board:
		make_board_png()
	elif args.capture:
		capture(config_path, args.exposure_scale, args.gain_offset_db)
	elif args.calibrate:
		calibrate(use_rational_model=not args.no_rational)
	else:
		ap.print_help()
