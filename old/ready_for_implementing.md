# Glitch Project: System State & Rigorous Validation Plan

## 1. System State Summary
All three nodes (Base ESP32, Arm ESP32, Camera ESP32-S3) are compile-ready with the following stabilized features:

*   **Camera (ESP32-S3):** Chunked HTTP streaming, mutex-protected telemetry (`g_data_mutex`, `g_platform_mutex`, `g_cam_mutex`), spinlock for ESP-NOW scan flags (`g_scan_mux`), and a dedicated QR task on Core 1 with 32KB stack.
*   **Base (ESP32):** Camera-guided autonomous alignment (`alignToQR`, `waitForCameraPose`), dynamic arm IK duration polling, Blynk telemetry with safety watchdog, and ESP-NOW centralized routing.
*   **Arm (ESP32):** `strcmp`-based ESP-NOW command dispatch, robust Inverse Kinematics (IK) with singularity/reachability checks, and smooth-step trajectory interpolation.

## 2. Pre-Testing Verifications (Already Correct & Verified in Codebase)
*   Autonomous mode cancellation works (instant `!autonomousMode` breaks + `Blynk.run()` polling loops).
*   Blocking `delay(9000)` replaced with `Blynk.run()` poll loop in autonomous sequence.
*   Arm ESP-NOW callback uses zero-allocation `strcmp` instead of Arduino `String`.
*   Arm `executeSyncMove` uses dynamic duration from `maxDelta * MS_PER_DEGREE` (clamped to 300ms min).
*   I2C encoder failure handles 5-consecutive-failure timeouts safely.
*   Encoder wrapping is safe on the two's complement ESP32 architecture.
*   Struct sizes match across all three nodes (`PoseReply=24B`, `ScanRequest=4B`).
*   Sobel threshold for platform detection correctly set to `160`.

## 3. Rigorous Baby-Step Validation Checklist

*Execute strictly in this order. Do not skip steps. Each step must pass before proceeding.*

### Phase 0: Safety (Bench, Disconnected)
*   [ ] **0.1** Inspect all I2C and servo wiring for shorts.
*   [ ] **0.2** Confirm Base motor driver board power rating matches PSU (expect 12V).
*   [ ] **0.3** Confirm Arm PCA9685 servos are powered independently via buck converter (NOT from ESP 5V pin).
*   [ ] **0.4** Camera ESP32-S3 powered via USB only (5V/500mA min).

### Phase 1: Boot & Network Initialization (Standalone Nodes)
*   [ ] **1.1 Camera Boot:** Flash & boot Camera. Serial prints "Ready" and `CAMERA MAC: xx:xx:xx:xx:xx:xx`. **Note the MAC.**
*   [ ] **1.2 Arm Boot:** Flash & boot Arm. Serial prints `ARM SYSTEM ONLINE`. **Note MAC.** Arm defaults to home position.
*   [ ] **1.3 Base Boot:** Flash & boot Base. Serial prints `BASE MAC: xx:xx:xx:xx:xx:xx`. **Note MAC.** Update `cameraAddress` and `armAddress` in `basewithBlynk.ino`.
*   [ ] **1.4 Hotspot Check:** Start laptop hotspot `hassan's-laptop-hotspot`. All three nodes connect and print `Connected - IP: 192.168.5.x`.
*   [ ] **1.5 Blynk Check:** Open `http://localhost:8080/admin` — local Blynk server admin page loads successfully.

### Phase 2: Camera Vision Standalone (Base & Arm OFF)
*   [ ] **2.1 Stream Render:** Open `http://<camera-ip>/stream` in browser. MJPEG video renders smoothly at ≥5 FPS.
*   [ ] **2.2 QR Read Accuracy:** Hold a 50×50mm QR code (R/G/B) 300mm away. `http://<camera-ip>/data` shows `qr_codes[].text` with confidence > 0.5. Pose (`tx, ty, tz, yaw`) matches physical reality.
*   [ ] **2.3 QR Occlusion:** Remove QR. Confidence drops, `estimated=true` flags, and eventually QR times out and disappears from JSON payload.
*   [ ] **2.4 Platform Edge Detection:** Place an 80×80mm black square on white background ~400mm away. `http://<camera-ip>/platform` returns `detected=true`, and `distance_mm` is within ±20% of reality.
*   [ ] **2.5 No-Target Verification:** Remove the square. `http://<camera-ip>/platform` returns `detected=false`.
*   [ ] **2.6 Telemetry Health:** `http://<camera-ip>/status` returns JSON with `camera_ok=true`, `qr_fps > 0`, and stable `free_heap`.

### Phase 3: Base Kinematics Standalone (Arm & Camera OFF)
*   [ ] **3.1 Motor Initialization:** Serial shows motor type initialized. No I2C errors.
*   [ ] **3.2 Manual Teleop:** Via Blynk app, press V1 (forward), V2 (back), V3 (left), V4 (right). Wheels start and stop instantly upon button release.
*   [ ] **3.3 PWM Proportional Control:** Rotate slider V7 (`Motor_speed`) from 0 → 80 → 0. Wheels respond proportionally.
*   [ ] **3.4 Rotation Accuracy:** Press V15 (rotate CW). Robot rotates ~90° and stops. Repeat x4. Verify drift accumulation (<15° error after full 360° rotation).
*   [ ] **3.5 I2C Stall Simulation:** Disconnect I2C encoder wire mid-movement. Verify robot aborts movement and triggers `forceStop()` after 5 failures.

### Phase 4: Base ↔ Arm ESP-NOW Integration (Camera OFF)
*   [ ] **4.1 ESP-NOW Handshake:** Power both Base and Arm. Base prints `ESPNOW: Base peer added`.
*   [ ] **4.2 Direct Command:** Press V16 ("BTC"). Serial shows BTC on Base, then Arm executes `BlueToCar()` sequence.
*   [ ] **4.3 Command Variations:** Test V17 ("RTC"), V18 ("GTC"), and V19 ("H"). Arm maneuvers correctly with dynamic speeds.
*   [ ] **4.4 Override Test:** Press V16, then immediately press V19 before sequence finishes. Arm should safely complete or abort (depending on execution state).
*   [ ] **4.5 Unreachable IK Override:** Send target coordinates intentionally out of reach. Verify Arm rejects coordinates via singularity/bounds check and prints `[!] ERROR: Unreachable`.

### Phase 5: Base ↔ Camera ESP-NOW Integration (Arm OFF)
*   [ ] **5.1 Peer Connection:** Power Base and Camera. Wait 10s for ESP-NOW. Base Serial shows periodic updates.
*   [ ] **5.2 Targeted Scan Trigger:** Place QR 300mm in front. Base Serial shows `pose_valid=1`. Remove QR. Base shows `pose_valid=0` within 3s.
*   [ ] **5.3 Telemetry Sync:** Move QR left/right. `yaw_deg` changes. Verify Blynk app V11-V14 (Confidence, Yaw, Color, Distance) update in real-time.
*   [ ] **5.4 Drop Recovery:** Power off Base, wait 5 seconds, power back on. Verify ESP-NOW auto-reconnects with Camera without restarting Camera.

### Phase 6: System-Wide Dashboard Integrity (All Nodes ON, Static)
*   [ ] **6.1 Data Aggregation:** Open `dashboard/index.html`. Verify MJPEG stream renders, QR data panel matches physical QR, and Platform panel matches physical square simultaneously.
*   [ ] **6.2 Memory Monitor:** Monitor `free_heap` on the dashboard over a 5-minute period. Verify no memory leaks occur during idle state.

### Phase 7: Closed-Loop Camera-Guided Alignment (Robot on Blocks)
*   [ ] **7.1 Alignment Trigger:** Place QR 500mm in front. Toggle V0 (Autonomous) ON.
*   [ ] **7.2 Scan Loop:** Base sends `[CAM] Scan request` repeatedly.
*   [ ] **7.3 Autonomous Abort:** Toggle V0 OFF mid-alignment. Robot stops instantly (<200ms).
*   [ ] **7.4 Yaw Correction:** Manually yaw robot 20° left. Alignment loop drives right. Robot correctly centers on QR.
*   [ ] **7.5 Distance Approach:** Push robot >300mm away. Robot drives forward until <200mm and stops.

### Phase 8: Full Autonomous Pickup Flow (Floor Testing)
*   [ ] **8.1 Sequence Start:** Place "RED" QR 500mm in front. Set `Motor_speed=40`. Toggle Autonomous ON.
*   [ ] **8.2 Visual Servoing:** Camera detects QR → Base strafes to center → Approaches → Checks color match → Sends "RTF".
*   [ ] **8.3 Arm Actuation:** Arm executes `RedToFloor()` sequence.
*   [ ] **8.4 Base Waiting State:** Base pauses and polls `Blynk.run()` while Arm acts (~9s window).
*   [ ] **8.5 Auto-Continuation:** After Arm homes, Base seeks next target (GREEN QR).
*   [ ] **8.6 Auto-Completion:** After 3rd target, Base sends "H", runs `forceStop()`, and prints `[AUTO] Autonomous pickup complete`. V0 toggles OFF.

### Phase 9: Edge Cases & Error Recovery Testing
*   [ ] **9.1 Vision Mismatch:** Show RED QR but place GREEN marker. Base skips and prints `Color mismatch`.
*   [ ] **9.2 Vision Timeout:** Cover camera lens during autonomous approach. Base prints `Camera scan timeout` and safely skips.
*   [ ] **9.3 Disconnection Resiliency:** Unplug Camera USB mid-run. Base triggers no-data timeout (3s) and safely skips.
*   [ ] **9.4 ESP-NOW Jamming:** Block Base antenna with foil. Base fails to send command (`OnDataSent FAILED`). Verify no crash or reboot loop.
*   [ ] **9.5 Hardware Stalls:** Physically block wheels during manual move. Confirm I2C/timeout aborts movement and motors un-power.
*   [ ] **9.6 Lighting Extremes:** Dim room to <50 lux. Test if QR decodes with >0.3 confidence. Shine flashlight at QR to test adaptive threshold bias.
*   [ ] **9.7 False Positives:** Place non-square obstacle 400mm from camera. Platform detector must return `detected=false`.

### Phase 10: 10-Minute Stress Test (Competition Simulation)
*   [ ] **10.1** Run 3 complete autonomous cycles back-to-back without restarting any nodes.
*   [ ] **10.2** Monitor Serial for "FAILED", "timeout", or "error" messages.
*   [ ] **10.3** Confirm Dashboard MJPEG stream doesn't freeze or stutter over time.
*   [ ] **10.4** Confirm Blynk server connection remains perfectly stable (no watchdog resets).
*   [ ] **10.5** Check Camera Serial: `[QR] processing times` must remain <100ms per frame.
*   [ ] **10.6** Thermals: Touch ESP32 heatsinks post-test. They should be warm, but not >60°C.
