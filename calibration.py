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
import os
import sys
import numpy as np
import cv2

# ---- ChArUco board definition (match your printed board) --------------------
SQUARES_X   = 10          # squares across
SQUARES_Y   = 7           # squares down
SQUARE_LEN  = 0.035       # square side in metres (measure your print!)
MARKER_LEN  = 0.026       # aruco marker side in metres
DICTIONARY  = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
IMG_DIR     = "calib_frames"

board    = cv2.aruco.CharucoBoard((SQUARES_X, SQUARES_Y), SQUARE_LEN, MARKER_LEN, DICTIONARY)
detector = cv2.aruco.CharucoDetector(board)


def make_board_png(path="charuco_board.png", px_per_square=240):
    """Render the board to print. Print FLAT on rigid backing; verify SQUARE_LEN after printing."""
    img = board.generateImage((SQUARES_X * px_per_square, SQUARES_Y * px_per_square))
    cv2.imwrite(path, img)
    print(f"wrote {path} -- print it, mount flat, then MEASURE a square and update SQUARE_LEN")


def capture():
    """Grab frames from the Basler and save the ones you like."""
    from pypylon import pylon
    os.makedirs(IMG_DIR, exist_ok=True)
    cam = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
    cam.Open()
    cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    conv = pylon.ImageFormatConverter()
    conv.OutputPixelFormat = pylon.PixelType_Mono8
    n = len(glob.glob(f"{IMG_DIR}/*.png"))
    print("SPACE = save frame, q = quit. Cover ALL edges & corners, tilt the board, vary distance.")
    while cam.IsGrabbing():
        res = cam.RetrieveResult(2000, pylon.TimeoutHandling_ThrowException)
        if res.GrabSucceeded():
            img = conv.Convert(res).GetArray()
            disp = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            cc, cids, _, _ = detector.detectBoard(img)
            if cids is not None and len(cids) > 0:
                cv2.aruco.drawDetectedCornersCharuco(disp, cc, cids)
            cv2.putText(disp, f"saved: {n}", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            cv2.imshow("capture", cv2.resize(disp, None, fx=0.6, fy=0.6))
            k = cv2.waitKey(1) & 0xFF
            if k == ord(" "):
                cv2.imwrite(f"{IMG_DIR}/frame_{n:03d}.png", img)
                n += 1
                print(f"saved frame_{n-1:03d}.png")
            elif k == ord("q"):
                break
        res.Release()
    cam.StopGrabbing(); cam.Close(); cv2.destroyAllWindows()
    print(f"{n} frames in {IMG_DIR}/")


def calibrate(use_rational_model=True):
    """Compute K + distortion from saved frames; save to calib.npz."""
    files = sorted(glob.glob(f"{IMG_DIR}/*.png"))
    if len(files) < 12:
        sys.exit(f"only {len(files)} frames -- grab ~20-40 covering the whole frame")

    objpoints, imgpoints, img_size = [], [], None
    for f in files:
        img = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        img_size = img.shape[::-1]  # (w, h)
        cc, cids, _, _ = detector.detectBoard(img)
        if cids is None or len(cids) < 6:
            print(f"  skip {os.path.basename(f)} (only {0 if cids is None else len(cids)} corners)")
            continue
        op, ip = board.matchImagePoints(cc, cids)
        if op is not None and len(op) >= 6:
            objpoints.append(op)
            imgpoints.append(ip)

    print(f"using {len(objpoints)} usable frames")
    # 4mm / ~79 deg is borderline-wide: the rational model (k4,k5,k6) fits the
    # edges noticeably better than the plain 5-coeff model. Fisheye is overkill here.
    flags = cv2.CALIB_RATIONAL_MODEL if use_rational_model else 0
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, img_size, None, None, flags=flags
    )

    print(f"\nRMS reprojection error: {rms:.4f} px   (aim for < ~0.3; >0.6 => recapture)")
    print("K =\n", K)
    print("dist =", dist.ravel())
    np.savez("calib.npz", K=K, dist=dist, img_size=img_size, rms=rms)
    print("\nsaved calib.npz")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--make-board", action="store_true")
    ap.add_argument("--capture", action="store_true")
    ap.add_argument("--calibrate", action="store_true")
    ap.add_argument("--no-rational", action="store_true", help="use plain 5-coeff model")
    a = ap.parse_args()
    if a.make_board:  make_board_png()
    elif a.capture:   capture()
    elif a.calibrate: calibrate(use_rational_model=not a.no_rational)
    else:             ap.print_help()
