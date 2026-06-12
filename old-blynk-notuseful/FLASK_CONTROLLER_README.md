# Glitch Controller — Python Flask + WebSocket

This is the **Blynk-free** controller stack. Replaces the Blynk local server
and the Blynk mobile app with:

- **Python Flask + Flask-SocketIO** running on the laptop (this `server.py`)
- **Phone browser** opens `controller.html` served by the same Flask app
- **Base ESP32** talks to Flask over plain TCP on port 9000

## Quick start

```powershell
# 1. Plug in the Base ESP32 (and the camera, optional)
# 2. Start the laptop hotspot: hassan's-laptop-hotspot / 12345678
# 3. Run:
.\StartGlitchServer.bat
# 4. On the phone (connected to the same hotspot), open:
http://192.168.5.1:5000
```

## Architecture

```
   Phone                Laptop                  ESP32 fleet
┌──────────┐    WS     ┌──────────┐     TCP     ┌──────────┐
│ Browser  │ ◄───────► │ server.py│  ◄───────► │ Base S3  │
│/5000     │  SocketIO │ Flask    │   port 9000 │ (firmware│
└──────────┘           │ + SocketIO│            │  v2)     │
                       └──────────┘             └────┬─────┘
                                                    │ ESP-NOW
                                               ┌────▼─────┐
                                               │ Arm ESP32│
                                               └──────────┘
```

## Wire protocol (line-delimited JSON, one message per line)

| Direction | Payload | Meaning |
|---|---|---|
| Phone → Server | `{"cmd":"MOVE","arg":"FWD"}` | Drive forward |
| Phone → Server | `{"cmd":"SPEED","arg":40}` | Set motor speed 0–80 |
| Phone → Server | `{"cmd":"ARM","arg":"BTC"}` | Send arm command |
| Phone → Server | `{"cmd":"AUTO","arg":"TOGGLE"}` | Autonomous mode |
| Base → Server | `{"type":"telemetry","yaw":12.3,"color":"R",...}` | Periodic state |
| Base → Server | `{"type":"log","msg":"..."}` | Debug log to UI |
| Base → Server | `GET_CMD\n` | Poll for latest command |
| Server → Base | `{"cmd":"MOVE","arg":"FWD"}\n` or `IDLE\n` | Reply to poll |

## Why this works for a senior project

- **Real-time bidirectional** — Socket.IO is full-duplex
- **Single source of truth** — server is the only thing the phone and ESP32 talk to
- **Stateless** — pull-based command retrieval means no missed packets
- **No vendor lock-in** — no Blynk, no MQTT broker, no Play Store dependency
- **Easy to extend** — add a new `cmd` is one handler on each side

## Files

| File | Purpose |
|---|---|
| `server.py` | Flask app: serves controller, bridges phone↔Base |
| `dashboard/controller.html` | Mobile UI (D-pad, sliders, telemetry) |
| `basewithFlask.ino` | Base ESP32 firmware (TCP client, Blynk removed) |
| `StartGlitchServer.bat` | One-click launcher (auto-installs deps) |
| `basewithBlynk.ino` | **Original** firmware, kept for fallback |

## Migration safety

The original `basewithBlynk.ino` is **untouched**. To revert:

```powershell
# in platformio.ini, change [env:base] build_src_filter to use basewithBlynk.ino
# OR just reflash with the original Blynk firmware
```

If the Flask version has a bug on demo day, you can flash the Blynk version
in 2 minutes and you're back online.

## Reference projects

- https://www.donskytech.com/esp32-robot-car-using-websockets/ — DonskyTech WebSocket car
- https://www.tobias-weis.de/control-a-raspberrypi-robot-using-flask-and-a-mobile-browser/ — Tobias Weis Flask robot
- https://github.com/eriklindstein/ESP32-http-websocket — ESP32 self-hosted UI
