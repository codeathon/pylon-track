"""
Flask server — camera MJPEG stream + ODrive + LabJack T4 REST API.
Run: pip install flask opencv-python pyserial labjack-ljm  then  python web_ui/server.py
"""

# ┌─────────────────────────────────────────────────────────────────────────────┐
# │                          CYCLE CONFIGURATION                                │
# │  Edit CYCLE_TURNS here, or change it live from the web UI.                  │
# └─────────────────────────────────────────────────────────────────────────────┘

CYCLE_TURNS     = 20.0   # ← change this

DAC0_OFF_START  = 15.0   # turn within cycle where DAC0 switches OFF
DAC0_OFF_END    = 17.0   # turn within cycle where DAC0 switches back ON
DAC1_OFF_START  = 18.0   # turn within cycle where DAC1 switches OFF
DAC1_OFF_END    = 20.0   # turn within cycle where DAC1 switches back ON

VEL_LIMIT       = 10.0   # turns/s — hard ceiling on motor speed

# ─────────────────────────────────────────────────────────────────────────────

import argparse
import csv
import datetime
import os
import serial
import serial.tools.list_ports
import threading
import time
import cv2
from flask import Flask, Response, jsonify, request, send_file
from pathlib import Path

app = Flask(__name__)

# ── Mutable config (editable via /config endpoints) ──────────────────────────

cfg_lock = threading.Lock()
config = {
    "cycle_turns":    CYCLE_TURNS,
    "dac0_off_start": DAC0_OFF_START,
    "dac0_off_end":   DAC0_OFF_END,
    "dac1_off_start": DAC1_OFF_START,
    "dac1_off_end":   DAC1_OFF_END,
    "vel_limit":      VEL_LIMIT,
    "target_cycles":  0,
}

def get_cfg():
    with cfg_lock:
        return dict(config)

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


class ODriveSerial:
    def __init__(self):
        self._ser      = None
        self._lock     = threading.Lock()
        self.port      = None
        self.state     = "DISCONNECTED"
        self.vel_sp    = 0.0
        self._position = None

    def connect(self, port: str, baud: int = 921600, skip_config: bool = False) -> bool:
        try:
            ser = serial.Serial(port, baud, timeout=0.2)
            with self._lock:
                self._ser = ser
                self.port = port
            if not skip_config:
                lim = get_cfg()["vel_limit"]
                self._send(f"w axis0.controller.config.control_mode {CONTROL_MODE_VELOCITY}")
                self._send(f"w axis0.controller.config.vel_limit {lim}")
                print(f"[odrive] applied config: velocity mode, vel_limit={lim}")
            else:
                print("[odrive] skipping config — using existing ODrive configuration")
            self.state     = "IDLE"
            self._position = None
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

    def enable(self):
        self._send(f"w axis0.requested_state {AXIS_STATE_CLOSED_LOOP_CONTROL}")
        self.state = "ARMED"

    def set_velocity(self, turns_per_s: float):
        lim = get_cfg()["vel_limit"]
        clamped = max(-lim, min(lim, turns_per_s))
        self._send(f"v 0 {clamped:.4f}")
        self.vel_sp = clamped
        self.state  = "RUNNING" if clamped != 0.0 else "ARMED"

    def set_vel_limit(self, limit: float):
        self._send(f"w axis0.controller.config.vel_limit {limit:.4f}")

    def stop(self):
        self._send("v 0 0")
        self._send(f"w axis0.requested_state {AXIS_STATE_IDLE}")
        self.vel_sp = 0.0
        self.state  = "IDLE"

    def estop(self):
        with self._lock:
            ser = self._ser
        if ser and ser.is_open:
            try:
                ser.write(b"v 0 0\nw axis0.requested_state 1\n")
                ser.flush()
            except serial.SerialException:
                pass
        self.vel_sp = 0.0
        self.state  = "FAULT"

    def get_position(self) -> float:
        with self._lock:
            ser = self._ser
        if not ser or not ser.is_open:
            return None
        try:
            ser.write(b"r axis0.pos_estimate\n")
            ser.flush()
            resp = ser.readline().decode(errors="ignore").strip()
            return float(resp) if resp else None
        except Exception:
            return None

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
        c = get_cfg()
        return {
            "connected": self.connected,
            "port":      self.port,
            "state":     self.state,
            "velocity":  self.vel_sp,
            "vel_limit": c["vel_limit"],
            "position":  self._position,
            "config":    c,
        }


# ── LabJack T4 ───────────────────────────────────────────────────────────────

try:
    from labjack import ljm as _ljm
    _LJM_AVAILABLE = True
except ImportError:
    _ljm = None
    _LJM_AVAILABLE = False

DAC_CHANNELS   = ["DAC0", "DAC1"]
DAC_ON_VOLTAGE = 5.0


class LabJackT4:
    def __init__(self):
        self._handle   = None
        self._lock     = threading.Lock()
        self.connected = False
        self._state    = {ch: False for ch in DAC_CHANNELS}

    def connect(self) -> bool:
        if not _LJM_AVAILABLE:
            print("[labjack] labjack-ljm not installed")
            return False
        try:
            with self._lock:
                self._handle = _ljm.openS("T4", "ANY", "ANY")
            for ch in DAC_CHANNELS:
                self._write(ch, DAC_ON_VOLTAGE)
                self._state[ch] = True
            self.connected = True
            print("[labjack] T4 connected")
            return True
        except Exception as e:
            print(f"[labjack] connect failed: {e}")
            return False

    def disconnect(self):
        with self._lock:
            if self._handle is not None:
                try:
                    for ch in DAC_CHANNELS:
                        _ljm.eWriteName(self._handle, ch, 0.0)
                    _ljm.close(self._handle)
                except Exception:
                    pass
                self._handle = None
        self.connected = False
        self._state = {ch: False for ch in DAC_CHANNELS}
        print("[labjack] disconnected")

    def set_channel(self, channel: str, on: bool) -> bool:
        if channel not in DAC_CHANNELS:
            return False
        ok = self._write(channel, DAC_ON_VOLTAGE if on else 0.0)
        if ok:
            self._state[channel] = on
        return ok

    def _write(self, channel: str, voltage: float) -> bool:
        with self._lock:
            if self._handle is None:
                return False
            try:
                _ljm.eWriteName(self._handle, channel, voltage)
                return True
            except Exception as e:
                print(f"[labjack] write {channel} error: {e}")
                self.connected = False
                return False

    def status(self) -> dict:
        return {
            "connected": self.connected,
            "channels":  {ch: self._state[ch] for ch in DAC_CHANNELS},
        }


# ── CSV Logger ────────────────────────────────────────────────────────────────

class DataLogger:
    def __init__(self):
        self._file      = None
        self._writer    = None
        self._lock      = threading.Lock()
        self.logging    = False   # file is open
        self.recording  = False   # actively writing rows
        self.filename   = None
        self.row_count      = 0
        self.session_cycles = 0

    def start(self, filename: str, notes: str = "") -> bool:
        with self._lock:
            if self.logging:
                self._close()
            try:
                self._file = open(filename, "w", newline="")
                if notes:
                    self._file.write(f"# {notes}\n")
                self._writer = csv.writer(self._file)
                self._writer.writerow([
                    "timestamp", "motor_pos_turns", "cycle_pos_turns",
                    "ferret_x_mm", "ferret_y_mm", "ferret_speed_mm_s",
                    "prey_x_mm",   "prey_y_mm",   "prey_speed_mm_s",
                    "distance_mm"
                ])
                self._file.flush()
                self.logging        = True
                self.recording      = True
                self.filename       = filename
                self.row_count      = 0
                self.session_cycles = 0
                print(f"[log] started → {filename}")
                return True
            except Exception as e:
                print(f"[log] failed to open {filename}: {e}")
                return False

    def pause(self):
        with self._lock:
            if self.logging:
                self.recording = False
                print(f"[log] paused ({self.row_count} rows)")

    def resume(self):
        with self._lock:
            if self.logging:
                self.recording = True
                print("[log] resumed")

    def write(self, motor_pos, cycle_pos,
              ferret_x=None, ferret_y=None, ferret_spd=None,
              prey_x=None,   prey_y=None,   prey_spd=None,
              distance=None):
        with self._lock:
            if not self.logging or not self.recording or self._writer is None:
                return
            ts = datetime.datetime.now().isoformat(timespec="milliseconds")
            self._writer.writerow([
                ts,
                f"{motor_pos:.4f}" if motor_pos is not None else "",
                f"{cycle_pos:.4f}" if cycle_pos is not None else "",
                f"{ferret_x:.1f}"  if ferret_x  is not None else "",
                f"{ferret_y:.1f}"  if ferret_y  is not None else "",
                f"{ferret_spd:.1f}" if ferret_spd is not None else "",
                f"{prey_x:.1f}"    if prey_x    is not None else "",
                f"{prey_y:.1f}"    if prey_y    is not None else "",
                f"{prey_spd:.1f}"  if prey_spd  is not None else "",
                f"{distance:.1f}"  if distance  is not None else "",
            ])
            self.row_count += 1
            if self.row_count % 50 == 0:
                self._file.flush()

    def stop(self):
        with self._lock:
            self._close()

    def _close(self):
        if self._file:
            try:
                self._file.flush()
                self._file.close()
            except Exception:
                pass
        self._file      = None
        self._writer    = None
        self.logging    = False
        self.recording  = False
        print(f"[log] stopped ({self.row_count} rows)")

    def status(self) -> dict:
        return {
            "logging":        self.logging,
            "recording":      self.recording,
            "filename":       self.filename,
            "row_count":      self.row_count,
            "session_cycles": self.session_cycles,
        }


# ── Motor → LabJack bridge ────────────────────────────────────────────────────

_bridge_state = {"prev_cycle_pos": None}  # mutable so session/start can reset it

def _motor_labjack_loop(od: ODriveSerial, lj: LabJackT4,
                        logger: DataLogger, stop_evt: threading.Event):
    while not stop_evt.is_set():
        if od.connected:
            pos = od.get_position()
            if pos is not None:
                od._position = pos
                c = get_cfg()
                cycle_turns = c["cycle_turns"]
                cycle_pos = pos % cycle_turns
                if cycle_pos < 0:
                    cycle_pos += cycle_turns

                # detect a full cycle wrap (position crossed 0 from high → low)
                prev = _bridge_state["prev_cycle_pos"]
                if prev is not None and prev > cycle_turns * 0.75 and cycle_pos < cycle_turns * 0.25:
                    logger.session_cycles += 1
                    target = c.get("target_cycles", 0)
                    if target > 0 and logger.session_cycles >= target and logger.recording:
                        print(f"[session] auto-stop after {logger.session_cycles} cycles")
                        od.stop()
                        logger.stop()
                _bridge_state["prev_cycle_pos"] = cycle_pos

                want_dac0 = not (c["dac0_off_start"] <= cycle_pos <= c["dac0_off_end"])
                want_dac1 = not (c["dac1_off_start"] <= cycle_pos <= c["dac1_off_end"])

                if lj.connected:
                    if want_dac0 != lj._state.get("DAC0", False):
                        lj.set_channel("DAC0", want_dac0)
                    if want_dac1 != lj._state.get("DAC1", False):
                        lj.set_channel("DAC1", want_dac1)

                logger.write(motor_pos=pos, cycle_pos=cycle_pos)

        stop_evt.wait(timeout=0.02)


# ── Singletons ────────────────────────────────────────────────────────────────

stream   = CameraStream()
odrive   = ODriveSerial()
labjack  = LabJackT4()
logger   = DataLogger()

_bridge_stop   = threading.Event()
_bridge_thread = threading.Thread(
    target=_motor_labjack_loop,
    args=(odrive, labjack, logger, _bridge_stop),
    daemon=True,
)
_bridge_thread.start()

# ── Routes — camera ───────────────────────────────────────────────────────────

def gen_frames():
    while True:
        jpeg = stream.get_jpeg()
        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n")
        time.sleep(0.033)


@app.route("/video_feed")
def video_feed():
    return Response(gen_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/")
def index():
    return (Path(__file__).parent / "index.html").read_text(encoding="utf-8")


# ── Routes — config ───────────────────────────────────────────────────────────

@app.route("/config")
def config_get():
    return jsonify(get_cfg())


@app.route("/config", methods=["POST"])
def config_set():
    data = request.get_json(force=True)
    with cfg_lock:
        for key in ("cycle_turns", "dac0_off_start", "dac0_off_end",
                    "dac1_off_start", "dac1_off_end", "vel_limit"):
            if key in data:
                config[key] = float(data[key])
    # Do NOT change ODrive's hardware vel_limit at runtime — that can fault the motor.
    # Server-side clamping in set_velocity handles the UI limit.
    # If the new limit is below the current setpoint, send a velocity command at the new cap.
    if odrive.connected and "vel_limit" in data:
        new_lim = config["vel_limit"]
        if abs(odrive.vel_sp) > new_lim:
            new_vel = new_lim if odrive.vel_sp >= 0 else -new_lim
            odrive._send(f"v 0 {new_vel:.4f}")
            odrive.vel_sp = new_vel
    return jsonify({"ok": True, "config": get_cfg(), **odrive.status()})


# ── Routes — ODrive ───────────────────────────────────────────────────────────

@app.route("/odrive/ports")
def od_ports():
    return jsonify([
        {"device": p.device, "description": p.description}
        for p in serial.tools.list_ports.comports()
    ])


@app.route("/odrive/status")
def od_status():
    return jsonify(odrive.status())


@app.route("/odrive/connect", methods=["POST"])
def od_connect():
    data        = request.get_json(force=True)
    port        = data.get("port", "")
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
    v = float(request.get_json(force=True).get("velocity", 0))
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


# ── Routes — LabJack ─────────────────────────────────────────────────────────

@app.route("/labjack/status")
def lj_status():
    return jsonify(labjack.status())


@app.route("/labjack/connect", methods=["POST"])
def lj_connect():
    ok = labjack.connect()
    return jsonify({"ok": ok, **labjack.status()})


@app.route("/labjack/disconnect", methods=["POST"])
def lj_disconnect():
    labjack.disconnect()
    return jsonify({"ok": True, **labjack.status()})


@app.route("/labjack/set", methods=["POST"])
def lj_set():
    if not labjack.connected:
        return jsonify({"ok": False, "error": "not connected"}), 400
    data    = request.get_json(force=True)
    channel = data.get("channel", "")
    on      = bool(data.get("on", False))
    ok      = labjack.set_channel(channel, on)
    return jsonify({"ok": ok, **labjack.status()})


# ── Routes — Logger ───────────────────────────────────────────────────────────

@app.route("/log/status")
def log_status():
    return jsonify(logger.status())


@app.route("/log/start", methods=["POST"])
def log_start():
    data     = request.get_json(force=True)
    filename = data.get("filename") or \
        f"session_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    ok = logger.start(filename)
    return jsonify({"ok": ok, **logger.status()})


@app.route("/log/stop", methods=["POST"])
def log_stop():
    logger.stop()
    return jsonify({"ok": True, **logger.status()})


@app.route("/log/download")
def log_download():
    fn = logger.filename
    if not fn or not Path(fn).exists():
        return jsonify({"error": "No file available"}), 404
    # Flush first in case the file is still open for writing
    with logger._lock:
        if logger._file:
            logger._file.flush()
    content = Path(fn).read_bytes()
    resp = Response(content, mimetype="text/csv")
    resp.headers["Content-Disposition"] = f'attachment; filename="{Path(fn).name}"'
    return resp


# ── Routes — Session ─────────────────────────────────────────────────────────

@app.route("/session/start", methods=["POST"])
def session_start():
    data          = request.get_json(force=True)
    vel           = float(data.get("velocity", 0))
    notes         = str(data.get("notes", "")).strip()
    target_cycles = max(0, int(data.get("target_cycles", 0)))
    filename      = data.get("filename") or \
        f"trial_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    if not odrive.connected:
        return jsonify({"ok": False, "error": "ODrive not connected"}), 400
    with cfg_lock:
        config["target_cycles"] = target_cycles
    _bridge_state["prev_cycle_pos"] = None  # reset cycle counter for new session
    odrive.enable()
    time.sleep(0.15)   # let ODrive finish transitioning to closed-loop before sending velocity
    odrive.set_velocity(vel)
    logger.start(filename, notes=notes)
    return jsonify({"ok": True, **odrive.status(), "log": logger.status()})


@app.route("/session/pause", methods=["POST"])
def session_pause():
    odrive.stop()
    logger.pause()
    return jsonify({"ok": True, **odrive.status(), "log": logger.status()})


@app.route("/session/resume", methods=["POST"])
def session_resume():
    data = request.get_json(force=True)
    vel  = float(data.get("velocity", 0))
    if not odrive.connected:
        return jsonify({"ok": False, "error": "ODrive not connected"}), 400
    odrive.enable()
    time.sleep(0.15)
    odrive.set_velocity(vel)
    logger.resume()
    return jsonify({"ok": True, **odrive.status(), "log": logger.status()})


@app.route("/session/end", methods=["POST"])
def session_end():
    odrive.stop()
    logger.stop()
    return jsonify({"ok": True, **odrive.status(), "log": logger.status()})


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera",    type=int,  default=0)
    parser.add_argument("--no-camera", action="store_true")
    parser.add_argument("--port",      type=str,  default=None)
    parser.add_argument("--baud",      type=int,  default=921600)
    parser.add_argument("--web-port",  type=int,  default=5000)
    args = parser.parse_args()

    if not args.no_camera:
        stream.start(index=args.camera)
    else:
        print("[camera] disabled")

    od_port = args.port
    if not od_port:
        for p in serial.tools.list_ports.comports():
            if "USB Serial" in p.description or "ODrive" in p.description:
                od_port = p.device
                print(f"[odrive] auto-detected on {od_port} ({p.description})")
                break
    if od_port:
        odrive.connect(od_port, args.baud)
    else:
        print("[odrive] not found — connect via the UI")

    labjack.connect()

    print(f"Web UI → http://localhost:{args.web_port}")
    try:
        app.run(host="0.0.0.0", port=args.web_port, threaded=True)
    finally:
        _bridge_stop.set()
        logger.stop()
        odrive.disconnect()
        labjack.disconnect()
        stream.stop()
