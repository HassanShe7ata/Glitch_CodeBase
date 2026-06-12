#!/usr/bin/env python3
"""
Glitch Controller — Flask + SocketIO bridge.

Phone browser  ──WebSocket──>  THIS SERVER  ──TCP──>  Base ESP32
                                              ──TCP──>  Arm ESP32  (optional)

Run:    pip install flask flask-socketio
        python server.py

Then open:  http://192.168.5.1:5000   on the phone
"""

import json
import socket
import threading
import time
from flask import Flask, send_from_directory
from flask_socketio import SocketIO

# ─── Config ────────────────────────────────────────────────────────────────
LISTEN_PORT        = 5000     # Flask + phone websocket
STATIC_FOLDER      = "dashboard"
ESP32_TCP_PORT     = 9000     # Base ESP32 connects back to us here
POLL_BASE_MS       = 150      # how often phone asks us for telemetry

# ─── Flask app ─────────────────────────────────────────────────────────────
app = Flask(__name__, static_folder=STATIC_FOLDER, static_url_path="")
app.config["SECRET_KEY"] = "glitch-robot-2026"
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# ─── Telemetry state (filled by ESP32 over TCP, read by phone over WS) ─────
state = {
    "connected":   False,
    "base_ip":     None,
    "last_seen":   0,
    "confidence":  0,
    "yaw":         0.0,
    "color":       "—",
    "distance_mm": 0,
    "motor_speed": 0,
    "free_heap":   0,
    "autonomous":  False,
}

# ─── TCP server: Base ESP32 connects in and pushes telemetry ───────────────
def tcp_server_thread():
    """Listen for Base ESP32 on ESP32_TCP_PORT. Receives JSON lines, broadcasts to phone."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", ESP32_TCP_PORT))
    srv.listen(1)
    print(f"[TCP] listening on :{ESP32_TCP_PORT} for Base ESP32")
    while True:
        try:
            conn, addr = srv.accept()
            print(f"[TCP] Base ESP32 connected from {addr}")
            state["connected"] = True
            state["base_ip"] = addr[0]
            buf = b""
            while True:
                chunk = conn.recv(1024)
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    try:
                        msg = json.loads(line.decode("utf-8", errors="ignore"))
                        kind = msg.get("type")
                        if kind == "telemetry":
                            for k in ("confidence", "yaw", "color", "distance_mm", "motor_speed", "free_heap", "autonomous"):
                                if k in msg:
                                    state[k] = msg[k]
                            state["last_seen"] = time.time()
                            socketio.emit("telemetry", state)
                        elif kind == "log":
                            print(f"[ESP32] {msg.get('msg','')}")
                            socketio.emit("esp_log", msg.get("msg", ""))
                    except Exception as e:
                        print(f"[TCP] bad json: {e}: {line!r}")
            conn.close()
            print("[TCP] Base ESP32 disconnected")
            state["connected"] = False
        except Exception as e:
            print(f"[TCP] server error: {e}")
            time.sleep(1)

threading.Thread(target=tcp_server_thread, daemon=True).start()

# ─── Last command queue (Base ESP32 will pull this when it connects) ───────
# Simpler: phone -> server -> Base via the same TCP socket, with a small
# request/response. We implement a pull model: Base sends "GET_CMD\n" every
# 100ms, we respond with the latest command as JSON.
latest_cmd = {"cmd": "IDLE", "ts": 0}
def set_command(cmd_dict):
    global latest_cmd
    latest_cmd = {"cmd": cmd_dict.get("cmd", "IDLE"), "arg": cmd_dict.get("arg", 0), "ts": time.time()}

# ─── Phone -> server: WebSocket events ──────────────────────────────────────
@socketio.on("connect")
def on_phone_connect():
    print("[WS] phone connected")
    socketio.emit("telemetry", state)

@socketio.on("cmd")
def on_cmd(data):
    """Phone sent a command. e.g. {"cmd":"MOVE","arg":"FWD"} or {"cmd":"V","pin":1,"val":1}"""
    set_command(data)
    print(f"[WS] cmd: {data}")

# ─── HTTP: serve the controller page ───────────────────────────────────────
@app.route("/")
def index():
    return send_from_directory(STATIC_FOLDER, "controller.html")

@app.route("/healthz")
def healthz():
    return {"ok": True, "esp32_connected": state["connected"]}

# ─── Boot ──────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print(f"[HTTP] serving {STATIC_FOLDER} on 0.0.0.0:{LISTEN_PORT}")
    print(f"        phone:  http://192.168.5.1:{LISTEN_PORT}")
    socketio.run(app, host="0.0.0.0", port=LISTEN_PORT, debug=False, allow_unsafe_werkzeug=True)
