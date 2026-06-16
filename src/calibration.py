#!/usr/bin/env python3
"""
Calibrate Basler a2A1920-160um + 4mm lens with a ChArUco board, then save
intrinsics (K) and distortion coefficients for undistortion.

The 4 mm lens (~79°) is wide enough that fisheye undistort usually fits the long
horizontal edges better than the plain rational polynomial model.

Requires: opencv-contrib-python >= 4.7  (CharucoDetector API), pypylon

Usage (from repo root):
  python src/calibration.py --make-board
  python src/calibration.py --capture
  python src/calibration.py --calibrate
  python src/calibration.py --preview          # raw vs undistorted grid check
"""

import argparse
import glob
import json
import os
import sys
from pathlib import Path

import cv2
import numpy as np

SQUARES_X = 10
SQUARES_Y = 7
SQUARE_LEN = 0.035
MARKER_LEN = 0.026
DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
IMG_DIR = str(REPO_ROOT / "calib_frames")
CALIB_NPZ = str(REPO_ROOT / "calib.npz")
PREVIEW_PNG = str(REPO_ROOT / "undistort_preview.png")
DEFAULT_CAMERA_CONFIG = str(SCRIPT_DIR / "camera_config.json")

MODEL_STANDARD = 0
MODEL_FISHEYE = 1

# Wider bands on left/right — the 1920 px sides are where barrel distortion shows.
EDGE_MARGIN_X = 0.18
EDGE_MARGIN_Y = 0.12
ZONE_GRID = 3
# alpha=0 over-flattens the centre while edges stay curved; ~0.25 balances both.
UNDISTORT_ALPHA = 0.25
MAX_CENTER_ONLY_VIEWS = 8
EDGE_VIEW_COPIES = 2
MIN_POINTS_PER_VIEW = 8
MIN_EDGE_POINTS_FOR_WEIGHT = 8


def make_board():
	return cv2.aruco.CharucoBoard(
		(SQUARES_X, SQUARES_Y), SQUARE_LEN, MARKER_LEN, DICTIONARY)


def make_detector():
	det_params = cv2.aruco.DetectorParameters()
	det_params.adaptiveThreshWinSizeMin = 3
	det_params.adaptiveThreshWinSizeMax = 33
	det_params.adaptiveThreshWinSizeStep = 4
	det_params.adaptiveThreshConstant = 5
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
		REPO_ROOT / "build/bin/camera_config.json",
		SCRIPT_DIR / "camera_config.json",
	):
		if candidate and candidate.is_file():
			return candidate
	return Path(DEFAULT_CAMERA_CONFIG)


def load_camera_config(path: Path) -> dict:
	with path.open() as handle:
		return json.load(handle)


def apply_camera_config(cam, cfg: dict, exposure_scale: float, gain_offset_db: float):
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
		cam.ExposureTime.SetValue(float(cfg["exposure_time_us"]) * exposure_scale)

	gain_auto = bool(cfg.get("gain_auto", False))
	cam.GainAuto.SetValue("Continuous" if gain_auto else "Off")
	if not gain_auto:
		cam.Gain.SetValue(float(cfg["gain_db"]) + gain_offset_db)

	if bool(cfg.get("frame_rate_enable", True)):
		cam.AcquisitionFrameRateEnable.SetValue(True)
		cam.AcquisitionFrameRate.SetValue(float(cfg.get("frame_rate_fps", 200.0)))
	else:
		cam.AcquisitionFrameRateEnable.SetValue(False)

	if cfg.get("trigger_mode", "Off") != "Off":
		raise SystemExit("calibration capture requires trigger_mode Off")

	print(
		f"camera: {cfg['width']}x{cfg['height']} "
		f"exposure×{exposure_scale:.2f} gain+{gain_offset_db:.1f}dB"
	)


def detect_board(gray: np.ndarray):
	clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
	return detector.detectBoard(clahe.apply(gray))


def empty_coverage() -> dict:
	return {
		"zones": set(),
		"edges": {k: False for k in ("left", "right", "top", "bottom")},
		"strips": {k: False for k in (
			"left_top", "left_mid", "left_bot",
			"right_top", "right_mid", "right_bot",
		)},
	}


def zone_index(x: float, y: float, width: int, height: int) -> tuple[int, int]:
	col = min(ZONE_GRID - 1, max(0, int(x / width * ZONE_GRID)))
	row = min(ZONE_GRID - 1, max(0, int(y / height * ZONE_GRID)))
	return row, col


def coverage_from_corners(corners, width: int, height: int) -> dict:
	cov = empty_coverage()
	if corners is None or len(corners) == 0:
		return cov

	margin_x = width * EDGE_MARGIN_X
	margin_y = height * EDGE_MARGIN_Y
	third = height / 3.0
	for pt in corners.reshape(-1, 2):
		x, y = float(pt[0]), float(pt[1])
		cov["zones"].add(zone_index(x, y, width, height))
		if x < margin_x:
			cov["edges"]["left"] = True
			if y < third:
				cov["strips"]["left_top"] = True
			elif y < 2 * third:
				cov["strips"]["left_mid"] = True
			else:
				cov["strips"]["left_bot"] = True
		if x > width - margin_x:
			cov["edges"]["right"] = True
			if y < third:
				cov["strips"]["right_top"] = True
			elif y < 2 * third:
				cov["strips"]["right_mid"] = True
			else:
				cov["strips"]["right_bot"] = True
		if y < margin_y:
			cov["edges"]["top"] = True
		if y > height - margin_y:
			cov["edges"]["bottom"] = True
	return cov


def merge_coverage(saved: dict, live: dict) -> dict:
	return {
		"zones": saved["zones"] | live["zones"],
		"edges": {k: saved["edges"][k] or live["edges"][k] for k in saved["edges"]},
		"strips": {k: saved["strips"][k] or live["strips"][k] for k in saved["strips"]},
	}


def is_wide_frame(width: int, height: int) -> bool:
	return width > height * 1.3


def coverage_complete(cov: dict, width: int, height: int) -> bool:
	base = (
		len(cov["zones"]) >= ZONE_GRID * ZONE_GRID
		and all(cov["edges"].values())
	)
	if not is_wide_frame(width, height):
		return base
	# 1920×960: require corners along the full length of left/right borders.
	return base and all(cov["strips"].values())


def print_coverage_report(cov: dict, width: int, height: int, prefix: str = ""):
	edges = ", ".join(f"{k}={'yes' if v else 'NO'}" for k, v in cov["edges"].items())
	msg = f"{prefix}zones {len(cov['zones'])}/{ZONE_GRID * ZONE_GRID}, edges: {edges}"
	if is_wide_frame(width, height):
		missing = [k for k, v in cov["strips"].items() if not v]
		msg += f", long-edge strips missing: {missing if missing else 'none'}"
	print(msg)


def draw_coverage_hud(disp, width: int, height: int, saved_cov: dict, live_cov: dict):
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

	mx = int(width * EDGE_MARGIN_X)
	my = int(height * EDGE_MARGIN_Y)
	cv2.rectangle(disp, (0, 0), (mx, height - 1), (255, 120, 0), 2)
	cv2.rectangle(disp, (width - mx, 0), (width - 1, height - 1), (255, 120, 0), 2)
	cv2.rectangle(disp, (0, 0), (width - 1, my), (200, 200, 0), 1)
	cv2.rectangle(disp, (0, height - my), (width - 1, height - 1), (200, 200, 0), 1)

	if is_wide_frame(width, height):
		for y in (int(height / 3), int(2 * height / 3)):
			cv2.line(disp, (0, y), (mx, y), (255, 120, 0), 1)
			cv2.line(disp, (width - mx, y), (width - 1, y), (255, 120, 0), 1)
		cv2.putText(
			disp, "LONG EDGES: fill orange L/R bands top/mid/bot",
			(20, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 120, 0), 2,
		)


def normalize_view_points(obj_pts, img_pts) -> tuple[np.ndarray, np.ndarray]:
	"""OpenCV Point3f/Point2f expect float32 (N,3) and (N,2), C-contiguous."""
	obj = np.ascontiguousarray(obj_pts, dtype=np.float32).reshape(-1, 3)
	img = np.ascontiguousarray(img_pts, dtype=np.float32).reshape(-1, 2)
	if obj.shape[0] != img.shape[0]:
		raise ValueError("object/image point count mismatch")
	if not np.isfinite(obj).all() or not np.isfinite(img).all():
		raise ValueError("non-finite calibration point")
	return obj, img


def build_standard_views(objpoints, imgpoints):
	obj_out, img_out = [], []
	for op, ip in zip(objpoints, imgpoints):
		try:
			o, p = normalize_view_points(op, ip)
		except ValueError:
			continue
		if len(o) < MIN_POINTS_PER_VIEW:
			continue
		obj_out.append(o)
		img_out.append(p)
	return obj_out, img_out


def load_frame_sets():
	files = sorted(glob.glob(f"{IMG_DIR}/*.png"))
	if len(files) < 12:
		sys.exit(f"only {len(files)} frames in {IMG_DIR}/ — need ~20-40")

	objpoints, imgpoints, img_size = [], [], None
	total_cov = empty_coverage()
	for path in files:
		img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
		img_size = img.shape[::-1]
		cc, cids, _, _ = detect_board(img)
		if cids is None or len(cids) < MIN_POINTS_PER_VIEW:
			print(f"  skip {os.path.basename(path)} ({0 if cids is None else len(cids)} corners)")
			continue
		obj_pts, img_pts = board.matchImagePoints(cc, cids)
		if obj_pts is None or img_pts is None:
			continue
		try:
			obj, img = normalize_view_points(obj_pts, img_pts)
		except ValueError:
			continue
		if len(obj) < MIN_POINTS_PER_VIEW:
			continue
		objpoints.append(obj)
		imgpoints.append(img)
		total_cov = merge_coverage(
			total_cov, coverage_from_corners(cc, img_size[0], img_size[1]))
	return objpoints, imgpoints, img_size, total_cov


def frame_edge_fraction(img_pts: np.ndarray, img_size: tuple[int, int]) -> float:
	w, h = img_size
	mx, my = w * EDGE_MARGIN_X, h * EDGE_MARGIN_Y
	pts = img_pts.reshape(-1, 2)
	if len(pts) == 0:
		return 0.0
	on_edge = (
		(pts[:, 0] < mx) | (pts[:, 0] > w - mx)
		| (pts[:, 1] < my) | (pts[:, 1] > h - my)
	)
	return float(np.mean(on_edge))


def view_is_degenerate(img_pts: np.ndarray, img_size: tuple[int, int]) -> bool:
	"""Fisheye needs spread-out corners; edge-only strips with few points are ill-conditioned."""
	pts = img_pts.reshape(-1, 2)
	if len(pts) < MIN_POINTS_PER_VIEW:
		return True
	xmin, ymin = pts.min(axis=0)
	xmax, ymax = pts.max(axis=0)
	w, h = img_size
	span = max((xmax - xmin) / w, (ymax - ymin) / h)
	return span < 0.06


def rebalance_for_edges(objpoints, imgpoints, img_size):
	"""Up-weight edge-rich full frames only — never pass edge-only corner subsets to fisheye."""
	out_obj, out_img = [], []
	center_only_kept = 0

	for op, ip in zip(objpoints, imgpoints):
		if view_is_degenerate(ip, img_size):
			print("  skip degenerate view (<8 points or corners too collinear)")
			continue

		edge_frac = frame_edge_fraction(ip, img_size)
		pts = ip.reshape(-1, 2)
		w, h = img_size
		mx, my = w * EDGE_MARGIN_X, h * EDGE_MARGIN_Y
		edge_count = int(np.sum(
			(pts[:, 0] < mx) | (pts[:, 0] > w - mx)
			| (pts[:, 1] < my) | (pts[:, 1] > h - my)
		))

		if edge_frac > 0.2 and edge_count >= MIN_EDGE_POINTS_FOR_WEIGHT:
			# Duplicate the whole frame — partial edge subsets break fisheye conditioning.
			for _ in range(EDGE_VIEW_COPIES + 1):
				out_obj.append(op.copy())
				out_img.append(ip.copy())
		elif edge_frac <= 0.2:
			if center_only_kept < MAX_CENTER_ONLY_VIEWS:
				out_obj.append(op)
				out_img.append(ip)
				center_only_kept += 1
			else:
				print("  drop centre-only view (dataset already has enough middle)")
		else:
			out_obj.append(op)
			out_img.append(ip)

	if len(out_obj) < 5:
		print(
			f"warning: rebalance left {len(out_obj)} views — using all "
			f"{len(objpoints)} raw views for calibration"
		)
		return objpoints, imgpoints

	print(
		f"rebalanced {len(objpoints)} -> {len(out_obj)} views "
		f"(full edge frames ×{EDGE_VIEW_COPIES + 1}, centre cap {MAX_CENTER_ONLY_VIEWS})"
	)
	return out_obj, out_img


def edge_band_rms(objpoints, imgpoints, k_mat, dist, img_size, model_id: int) -> dict:
	"""Mean reprojection error in left/right edge bands (where long-edge distortion shows)."""
	w, h = img_size
	mx = w * EDGE_MARGIN_X
	bands = {"left": [], "right": [], "center": []}
	for obj_pts, img_pts in zip(objpoints, imgpoints):
		op, ip = normalize_view_points(obj_pts, img_pts)
		if model_id == MODEL_FISHEYE:
			proj, _ = cv2.fisheye.projectPoints(
				op.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), k_mat, dist)
			proj = proj.reshape(-1, 2)
		else:
			proj, _ = cv2.projectPoints(
				op.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), k_mat, dist)
			proj = proj.reshape(-1, 2)
		err = np.linalg.norm(proj - ip, axis=1)
		for e, (x, _y) in zip(err, ip):
			if x < mx:
				bands["left"].append(e)
			elif x > w - mx:
				bands["right"].append(e)
			else:
				bands["center"].append(e)
	return {k: float(np.mean(v)) if v else float("nan") for k, v in bands.items()}


def calibrate_standard(objpoints, imgpoints, img_size, rational: bool):
	obj, img = build_standard_views(objpoints, imgpoints)
	if len(obj) < 5:
		raise cv2.error(f"standard calibrate needs >=5 views, got {len(obj)}")
	flags = cv2.CALIB_RATIONAL_MODEL if rational else 0
	# OpenCV Python bindings accept either (N,3) or (N,1,3) Point3f layouts.
	layouts = (
		("Nx3", lambda o, p: (o, p)),
		("Nx1x3", lambda o, p: (o.reshape(-1, 1, 3), p.reshape(-1, 1, 2))),
	)
	last_err = None
	for label, layout in layouts:
		try:
			o_cv = [np.ascontiguousarray(layout(o, p)[0]) for o, p in zip(obj, img)]
			i_cv = [np.ascontiguousarray(layout(o, p)[1]) for o, p in zip(obj, img)]
			rms, k_mat, dist, _, _ = cv2.calibrateCamera(
				o_cv, i_cv, img_size, None, None, flags=flags)
			print(f"  standard calibrate OK ({label}, {len(o_cv)} views)")
			return rms, k_mat, dist, MODEL_STANDARD
		except cv2.error as exc:
			last_err = exc
			print(f"  standard layout {label} failed: {exc}")
	if obj:
		print(
			f"  debug view0: obj shape={obj[0].shape} dtype={obj[0].dtype} "
			f"img shape={img[0].shape} dtype={img[0].dtype}"
		)
	raise last_err if last_err is not None else cv2.error("standard calibration failed")


def make_initial_k(img_size: tuple[int, int]) -> np.ndarray:
	w, h = img_size
	focal = float(max(w, h)) * 0.85
	k = np.zeros((3, 3), dtype=np.float64)
	k[0, 0] = focal
	k[1, 1] = focal
	k[0, 2] = w / 2.0
	k[1, 2] = h / 2.0
	k[2, 2] = 1.0
	return k


def prepare_fisheye_views(objpoints, imgpoints, img_size):
	obj, img = [], []
	for op, ip in zip(objpoints, imgpoints):
		o, p = normalize_view_points(op, ip)
		if view_is_degenerate(p, img_size):
			continue
		obj.append(o.reshape(-1, 1, 3).astype(np.float32))
		img.append(p.reshape(-1, 1, 2).astype(np.float32))
	if len(obj) < 5:
		raise cv2.error(f"fisheye needs >=5 non-degenerate views, got {len(obj)}")
	return obj, img


def calibrate_fisheye(objpoints, imgpoints, img_size):
	obj, img = prepare_fisheye_views(objpoints, imgpoints, img_size)
	criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
	# CALIB_CHECK_COND aborts on ill-conditioned Jacobians — retry without it.
	attempts = (
		(cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_FIX_SKEW, False),
		(cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_FIX_SKEW, True),
		(cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC, True),
	)
	last_err = None
	for flags, use_guess in attempts:
		k_mat = make_initial_k(img_size) if use_guess else np.eye(3, dtype=np.float64)
		dist = np.zeros((4, 1), dtype=np.float64)
		try:
			rms, k_mat, dist, _, _ = cv2.fisheye.calibrate(
				obj, img, img_size, k_mat, dist, None, None, flags, criteria)
			return rms, k_mat, dist.reshape(1, 4), MODEL_FISHEYE
		except cv2.error as exc:
			last_err = exc
			print(f"    fisheye retry ({exc})")
	if last_err is not None:
		raise last_err
	raise cv2.error("fisheye calibration failed")


def model_score(edge: dict, overall_rms: float) -> float:
	"""Prefer models that fit edges, not just the already-flat centre."""
	worst_edge = max(edge.get("left", overall_rms), edge.get("right", overall_rms))
	centre = edge.get("center", overall_rms)
	# Penalise when centre error is much lower than edges (over-flat middle).
	centre_skew = max(0.0, worst_edge - centre) * 0.35
	return worst_edge + centre_skew


def pick_model(objpoints, imgpoints, img_size, model: str, rational: bool):
	fit_obj, fit_img = rebalance_for_edges(objpoints, imgpoints, img_size)
	candidates = []
	if model in ("auto", "fisheye"):
		try:
			rms, k, d, mid = calibrate_fisheye(fit_obj, fit_img, img_size)
			edge = edge_band_rms(objpoints, imgpoints, k, d, img_size, mid)
			score = model_score(edge, rms)
			candidates.append((score, rms, k, d, mid, "fisheye", edge))
			print(
				f"  fisheye RMS {rms:.4f}  "
				f"L/R/C {edge['left']:.3f}/{edge['right']:.3f}/{edge['center']:.3f}"
			)
		except cv2.error as exc:
			print(f"  fisheye failed: {exc}")
	if model in ("auto", "standard"):
		try:
			rms, k, d, mid = calibrate_standard(fit_obj, fit_img, img_size, rational)
			edge = edge_band_rms(objpoints, imgpoints, k, d, img_size, mid)
			score = model_score(edge, rms)
			candidates.append((score, rms, k, d, mid, "standard", edge))
			print(
				f"  standard RMS {rms:.4f}  "
				f"L/R/C {edge['left']:.3f}/{edge['right']:.3f}/{edge['center']:.3f}"
			)
		except cv2.error as exc:
			print(f"  standard failed: {exc}")
	if not candidates:
		sys.exit(
			"calibration failed for all models — try:\n"
			"  python src/calibration.py --calibrate --model standard\n"
			"  or add more frames with >=8 spread-out corners per view"
		)
	candidates.sort(key=lambda item: item[0])
	score, rms, k, d, mid, name, edge = candidates[0]
	print(f"selected {name} (score {score:.3f} px — edges weighted over centre)")
	return rms, k, d, mid, name, edge


def save_calib_npz(k_mat, dist, img_size, rms, model_id, edge_err, alpha):
	np.savez(
		CALIB_NPZ,
		K=k_mat,
		dist=dist,
		img_size=np.array(img_size),
		rms=np.array(rms),
		model_id=np.array(model_id),
		undistort_alpha=np.array(alpha),
		edge_err_left=np.array(edge_err.get("left", np.nan)),
		edge_err_right=np.array(edge_err.get("right", np.nan)),
	)
	print(f"saved {CALIB_NPZ}")


def undistort_image(gray, k_mat, dist, model_id: int, alpha: float):
	h, w = gray.shape
	if model_id == MODEL_FISHEYE:
		new_k = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
			k_mat, dist, (w, h), np.eye(3), balance=alpha, new_size=(w, h))
		map1, map2 = cv2.fisheye.initUndistortRectifyMap(
			k_mat, dist, np.eye(3), new_k, (w, h), cv2.CV_16SC2)
	else:
		new_k, _ = cv2.getOptimalNewCameraMatrix(k_mat, dist, (w, h), alpha, (w, h))
		map1, map2 = cv2.initUndistortRectifyMap(
			k_mat, dist, None, new_k, (w, h), cv2.CV_16SC2)
	return cv2.remap(gray, map1, map2, cv2.INTER_LINEAR)


def draw_grid(bgr, step=80):
	h, w = bgr.shape[:2]
	for x in range(0, w, step):
		cv2.line(bgr, (x, 0), (x, h - 1), (0, 255, 255), 1)
	for y in range(0, h, step):
		cv2.line(bgr, (0, y), (w - 1, y), (0, 255, 255), 1)


def make_board_png(path=None, px_per_square=240):
	if path is None:
		path = str(REPO_ROOT / "charuco_board.png")
	img = board.generateImage((SQUARES_X * px_per_square, SQUARES_Y * px_per_square))
	cv2.imwrite(path, img)
	print(f"wrote {path}")


def capture(camera_config: Path, exposure_scale: float, gain_offset_db: float):
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
	saved_cov = empty_coverage()
	width = int(cfg["width"])
	height = int(cfg["height"])

	print(
		"SPACE=save, q=quit.\n"
		"For 1920×960: slide the board along the ORANGE left/right bands — top, middle, "
		"and bottom of each long edge. Only a corner of the board needs to be visible.\n"
		f"config: {camera_config}"
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
		ready = coverage_complete(saved_cov, width, height)
		cv2.putText(
			disp,
			f"saved:{n} corners:{0 if cids is None else len(cids)} {'READY' if ready else 'fill L/R strips'}",
			(20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2,
		)
		cv2.imshow("capture", cv2.resize(disp, None, fx=0.55, fy=0.55))
		key = cv2.waitKey(1) & 0xFF

		if key == ord(" "):
			if cids is None or len(cids) < MIN_POINTS_PER_VIEW:
				print(f"  skip — need >={MIN_POINTS_PER_VIEW} spread-out corners")
			else:
				if is_wide_frame(width, height):
					missing = [k for k, v in live_cov["strips"].items() if not v]
					if missing:
						print(f"  warning: missing long-edge strips {missing}")
				if cids is not None and len(cids) >= MIN_POINTS_PER_VIEW:
					_, img_pts = board.matchImagePoints(cc, cids)
					if img_pts is not None:
						ip = np.ascontiguousarray(img_pts, dtype=np.float32).reshape(-1, 2)
						if frame_edge_fraction(ip, (width, height)) < 0.15:
							print(
								"  note: centre-heavy frame — you have enough middle; "
								"prioritise orange L/R edge strips"
							)
				cv2.imwrite(f"{IMG_DIR}/frame_{n:03d}.png", img)
				n += 1
				saved_cov = merge_coverage(saved_cov, live_cov)
				print(f"saved frame_{n - 1:03d}.png")
				print_coverage_report(saved_cov, width, height, "  ")
		elif key == ord("q"):
			res.Release()
			break
		res.Release()

	cam.StopGrabbing()
	cam.Close()
	cv2.destroyAllWindows()
	print(f"{n} frames in {IMG_DIR}/")
	print_coverage_report(saved_cov, width, height, "final: ")
	if not coverage_complete(saved_cov, width, height):
		print("WARNING: long-edge coverage incomplete — expect curved borders after undistort.")


def calibrate(model: str, rational: bool, alpha: float):
	objpoints, imgpoints, img_size, total_cov = load_frame_sets()
	w, h = img_size
	print(f"using {len(objpoints)} frames on {w}x{h}")
	print_coverage_report(total_cov, w, h, "dataset: ")
	if not coverage_complete(total_cov, w, h):
		print("WARNING: dataset missing long-edge strips — fisheye fit will extrapolate at borders.")

	rms, k_mat, dist, model_id, model_name, edge_err = pick_model(
		objpoints, imgpoints, img_size, model, rational)
	print(f"\nRMS {rms:.4f} px  model={model_name}  alpha={alpha}")
	print(
		f"band error  left={edge_err['left']:.3f}  right={edge_err['right']:.3f}  "
		f"centre={edge_err['center']:.3f} px"
	)
	if edge_err["center"] < 0.5 * min(edge_err["left"], edge_err["right"]):
		print(
			"WARNING: centre fits much better than edges — undistort will look flat "
			"in the middle but curved elsewhere. Add more orange-band edge frames."
		)
	print("K =\n", k_mat)
	print("dist =", dist.ravel())
	save_calib_npz(k_mat, dist, img_size, rms, model_id, edge_err, alpha)
	preview_undistort(img_size, k_mat, dist, model_id, alpha)


def preview_undistort(img_size, k_mat=None, dist=None, model_id=None, alpha=None):
	if k_mat is None:
		data = np.load(CALIB_NPZ)
		k_mat, dist = data["K"], data["dist"]
		img_size = tuple(int(x) for x in data["img_size"])
		model_id = int(data.get("model_id", MODEL_STANDARD))
		alpha = float(data.get("undistort_alpha", UNDISTORT_ALPHA))

	files = sorted(glob.glob(f"{IMG_DIR}/*.png"))
	if not files:
		sys.exit(f"no frames in {IMG_DIR}/ for preview")
	img = cv2.imread(files[len(files) // 2], cv2.IMREAD_GRAYSCALE)
	und = undistort_image(img, k_mat, dist, model_id, alpha)
	raw_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
	und_bgr = cv2.cvtColor(und, cv2.COLOR_GRAY2BGR)
	draw_grid(raw_bgr)
	draw_grid(und_bgr)
	combo = np.hstack([raw_bgr, und_bgr])
	cv2.putText(combo, "raw", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2)
	cv2.putText(combo, "undistorted", (img_size[0] + 20, 40),
		cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2)
	out = PREVIEW_PNG
	cv2.imwrite(out, combo)
	print(f"wrote {out} — yellow grid lines should be straight, especially on long edges")


if __name__ == "__main__":
	ap = argparse.ArgumentParser()
	ap.add_argument("--make-board", action="store_true")
	ap.add_argument("--capture", action="store_true")
	ap.add_argument("--calibrate", action="store_true")
	ap.add_argument("--preview", action="store_true", help="raw|undistorted grid from calib.npz")
	ap.add_argument("--model", choices=("auto", "fisheye", "standard"), default="standard")
	ap.add_argument("--no-rational", action="store_true")
	ap.add_argument("--camera-config", default="")
	ap.add_argument("--exposure-scale", type=float, default=2.5)
	ap.add_argument("--gain-offset-db", type=float, default=4.0)
	ap.add_argument("--undistort-alpha", type=float, default=UNDISTORT_ALPHA)
	args = ap.parse_args()
	config_path = resolve_camera_config(args.camera_config)

	if args.make_board:
		make_board_png()
	elif args.capture:
		capture(config_path, args.exposure_scale, args.gain_offset_db)
	elif args.calibrate:
		calibrate(args.model, not args.no_rational, args.undistort_alpha)
	elif args.preview:
		preview_undistort(None)
	else:
		ap.print_help()
