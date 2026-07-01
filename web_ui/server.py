"""
Flask server — camera MJPEG stream + ODrive motor control REST API.
Run: pip install flask opencv-python pyserial  then  python web_ui/server.py [--port /dev/ttyACM0]
"""

import argparse
import cv2
import serial
import serial.tools.list_ports
import threading
import time
from flask import Flask, Response, jsonify, request
from pathlib import Path

app = Flask(__name__)

# ── Camera ────────────────────────────────────────────────────────────────────

class CameraStream:
    def __init__(self):
        self.cap     = None
        self.frame   = None
        self.lock    = threading.Lock()
        self.running = False

    def start(self, index=0):
        self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            print(f"[camera] Could not open camera {index} — showing placeholder")
            self.cap = None
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self.running:
            if self.cap and self.cap.isOpened():
                ok, frame = self.cap.read()
                if ok:
                    with self.lock:
                        self.frame = frame
            time.sleep(0.016)

    def get_jpeg(self):
        with self.lock:
            frame = self.frame
        if frame is None:
            frame = _placeholder_frame()
        _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return buf.tobytes()

    def stop(self):
        self.running = False
        if self.cap:
            self.cap.release()


def _placeholder_frame():
    import numpy as np
    img = np.zeros((480, 640, 3), dtype="uint8")
    img[:] = (30, 30, 30)
    cv2.putText(img, "No Camera", (220, 230), cv2.FONT_HERSHEY_SIMPLEX,
                1.2, (120, 120, 120), 2)
    cv2.putText(img, "Connect a camera and restart", (130, 270),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (80, 80, 80), 1)
    return img


# ── ODrive ────────────────────────────────────────────────────────────────────

AXIS_STATE_IDLE                = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8
CONTROL_MODE_VELOCITY          = 2
VEL_LIMIT                      = 10.0   # turns/s — hard ceiling


class ODriveSerial:
    """Thin ASCII-protocol wrapper around an ODrive connected via USB-serial."""

    def __init__(self):
        self._ser   = None
        self._lock  = threading.Lock()
        self.port   = None
        self.state  = "DISCONNECTED"   # DISCONNECTED | IDLE | ARMED | RUNNING | FAULT
        self.vel_sp = 0.0

    # ── connection ──────────────────────────────────────────────────────────

    def connect(self, port: str, baud: int = 921600, skip_config: bool = False) -> bool:
        try:
            ser = serial.Serial(port, baud, timeout=0.2)
            with self._lock:
                self._ser = ser
                self.port = port
            if not skip_config:
                self._send(f"w axis0.controller.config.control_mode {CONTROL_MODE_VELOCITY}")
                self._send(f"w axis0.controller.config.vel_limit {VEL_LIMIT}")
                print(f"[odrive] applied config: velocity mode, vel_limit={VEL_LIMIT}")
            else:
                print(f"[odrive] skipping config — using existing ODrive configuration")
            self.state = "IDLE"
            print(f"[odrive] connected on {port}")
            return True
        except serial.SerialException as e:
            print(f"[odrive] cannot open {port}: {e}")
            self.state = "DISCONNECTED"
            return False

    def disconnect(self):
        self.stop()
        with self._lock:
            if self._ser and self._ser.is_open:
                self._ser.close()
            self._ser = None
        self.state = "DISCONNECTED"
        print("[odrive] disconnected")

    # ── control ──────────────────────────────────────────────────────────────

    def enable(self):
        """Put axis into CLOSED_LOOP_CONTROL."""
        self._send(f"w axis0.requested_state {AXIS_STATE_CLOSED_LOOP_CONTROL}")
        self.state = "ARMED"

    def set_velocity(self, turns_per_s: float):
        """Set velocity (clamped to ±VEL_LIMIT)."""
        clamped = max(-VEL_LIMIT, min(VEL_LIMIT, turns_per_s))
        self._send(f"v 0 {clamped:.4f}")
        self.vel_sp = clamped
        self.state  = "RUNNING" if clamped != 0.0 else "ARMED"

    def stop(self):
        """Zero velocity and return axis to IDLE."""
        self._send("v 0 0")
        self._send(f"w axis0.requested_state {AXIS_STATE_IDLE}")
        self.vel_sp = 0.0
        self.state  = "IDLE"

    def estop(self):
        """Immediate zero + idle — writes directly without waiting for the lock."""
        with self._lock:
            ser = self._ser
        if ser and ser.is_open:
            try:
                # Write both commands back-to-back in one syscall for minimum latency
                ser.write(b"v 0 0\nw axis0.requested_state 1\n")
                ser.flush()
            except serial.SerialException:
                pass
        self.vel_sp = 0.0
        self.state  = "FAULT"

    # ── internal ─────────────────────────────────────────────────────────────

    def _send(self, cmd: str):
        with self._lock:
            if self._ser and self._ser.is_open:
                try:
                    self._ser.write((cmd + "\n").encode())
                except serial.SerialException as e:
                    print(f"[odrive] write error: {e}")
                    self.state = "FAULT"

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._ser is not None and self._ser.is_open

    def status(self) -> dict:
        return {
            "connected": self.connected,
            "port":      self.port,
            "state":     self.state,
            "velocity":  self.vel_sp,
            "vel_limit": VEL_LIMIT,
        }


# ── Singletons ────────────────────────────────────────────────────────────────

stream = CameraStream()
odrive = ODriveSerial()

# ── Routes — camera ───────────────────────────────────────────────────────────

def gen_frames():
    while True:
        jpeg = stream.get_jpeg()
        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n")
        time.sleep(0.033)


@app.route("/video_feed")
def video_feed():
    return Response(gen_frames(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/")
def index():
    return (Path(__file__).parent / "index.html").read_text(encoding="utf-8")


# ── Routes — ODrive ───────────────────────────────────────────────────────────

@app.route("/odrive/ports")
def od_ports():
    ports = [
        {"device": p.device, "description": p.description}
        for p in serial.tools.list_ports.comports()
    ]
    return jsonify(ports)


@app.route("/odrive/status")
def od_status():
    return jsonify(odrive.status())


@app.route("/odrive/connect", methods=["POST"])
def od_connect():
    data        = request.get_json(force=True)
    port        = data.get("port", "/dev/ttyACM0")
    skip_config = data.get("skip_config", False)
    ok          = odrive.connect(port, skip_config=skip_config)
    return jsonify({"ok": ok, **odrive.status()})


@app.route("/odrive/disconnect", methods=["POST"])
def od_disconnect():
    odrive.disconnect()
    return jsonify({"ok": True, **odrive.status()})


@app.route("/odrive/enable", methods=["POST"])
def od_enable():
    if not odrive.connected:
        return jsonify({"ok": False, "error": "not connected"}), 400
    odrive.enable()
    return jsonify({"ok": True, **odrive.status()})


@app.route("/odrive/velocity", methods=["POST"])
def od_velocity():
    if not odrive.connected:
        return jsonify({"ok": False, "error": "not connected"}), 400
    data = request.get_json(force=True)
    v    = float(data.get("velocity", 0))
    odrive.set_velocity(v)
    return jsonify({"ok": True, **odrive.status()})


@app.route("/odrive/stop", methods=["POST"])
def od_stop():
    odrive.stop()
    return jsonify({"ok": True, **odrive.status()})


@app.route("/odrive/estop", methods=["POST"])
def od_estop():
    odrive.estop()
    return jsonify({"ok": True, **odrive.status()})


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int,   default=0,              help="OpenCV camera index")
    parser.add_argument("--no-camera", action="store_true",             help="Disable camera (show placeholder only)")
    parser.add_argument("--port",   type=str,   default=None,           help="ODrive serial port (e.g. /dev/ttyACM0)")
    parser.add_argument("--baud",   type=int,   default=921600,         help="Serial baud rate")
    parser.add_argument("--web-port", type=int, default=5000,           help="HTTP server port")
    args = parser.parse_args()

    if not args.no_camera:
        stream.start(index=args.camera)
    else:
        print("[camera] disabled — showing placeholder")

    if args.port:
        odrive.connect(args.port, args.baud)

    print(f"Web UI running at http://localhost:{args.web_port}")
    if args.port:
        print(f"ODrive on {args.port} @ {args.baud} baud  vel_limit={VEL_LIMIT} turns/s")
    else:
        print("ODrive: no --port given; connect via the UI")

    try:
        app.run(host="0.0.0.0", port=args.web_port, threaded=True)
    finally:
        odrive.disconnect()
        stream.stop()
