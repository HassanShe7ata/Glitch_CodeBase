# Glitch Dashboard — Data Display

Real-time sensor data and camera feed dashboard for the Glitch mecanum robot.

## How to Use

1. **Start the laptop hotspot** (`hassan's-laptop-hotspot`)
2. **Power on** the ESP32 base, arm, and camera
3. **Open** `index.html` in Chrome or Edge on the laptop
4. **Enter** the camera's IP address (check Serial Monitor — it prints `Connected - IP: 192.168.5.x` on boot)
5. **Press** the ▶ Connect button (or Ctrl+Enter)

## What It Shows

| Panel | Description |
|-------|-------------|
| Camera Stream | Live MJPEG feed from ESP32-S3 camera (port 81) |
| QR Detection | Detected text, decode status, confidence, age |
| Pose Estimation | Translation (X/Y/Z mm) and rotation (roll/pitch/yaw °) |
| Yaw Indicator | Visual bar showing QR left/right offset for strafing |
| Platform Detection | Square platform position, distance, size, angle, confidence |
| System Health | Free heap, PSRAM, QR FPS, tracker status |
| Motor Speed | Ring gauge reading motor speed from Blynk REST API |
| Event Log | Timestamped log of all dashboard events |

## Configuration

All settings are saved automatically to browser localStorage:

- **Camera IP**: The ESP32-S3 camera's IP on the shared hotspot
- **Blynk Server**: Laptop IP (`192.168.5.1`)
- **Port**: Blynk Legacy server port (`8080`)
- **Auth Token**: Your Blynk project auth token

## Network Requirements

All devices must be on the same WiFi network (`hassan's-laptop-hotspot`):
- Laptop (runs hotspot + Blynk server + opens dashboard)
- ESP32 Base
- ESP32 Arm
- ESP32-S3 Camera

## Keyboard Shortcuts

- `Ctrl+Enter` — Connect
- `Escape` — Disconnect
