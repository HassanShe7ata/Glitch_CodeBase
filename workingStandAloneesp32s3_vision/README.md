# ESP32-S3 Vision: On-Device QR Pose, Laptop Visualization

This repo is now in the current architecture where ESP32 does all QR and pose compute, while PC is used for visualization, logging, and scenario evaluation.

## What This Version Does

1. Detects QR on ESP32-S3 using quirc.
2. Decodes payload on-device when ECC succeeds.
3. Estimates 6-DOF pose on-device from QR corners.
4. Maintains a temporal tracker with confidence decay when observations drop.
5. Publishes JSON telemetry via HTTP.
6. Provides laptop overlay + terminal logging tools and scenario metrics scripts.

## General Pipeline Diagram

```mermaid
flowchart LR
	 A[Camera Frame RGB565] --> B[ESP32 Preprocess]
	 B --> C[Multi-pass QR Scan]
	 C --> D[Payload Decode + Pose Solve]
	 D --> E[Probabilistic Observation Gate]
	 E --> F[Temporal Tracker
	 observed or estimated]
	 F --> G[/data JSON]
	 F --> H[/status JSON]
	 A --> I[/capture JPEG]
	 A --> J[/stream MJPEG]
	 G --> K[PC Data Poller]
	 I --> L[PC Capture Viewer]
	 J --> M[PC Stream Viewer]
	 K --> N[Terminal Logs + Overlay + Scenario Metrics]
	 L --> N
	 M --> N
```

## Detailed Pipeline By Stage (Beginner Friendly + Code Accurate)

All firmware stages below are implemented in firmware/cam_stream/src/main.cpp.

### Pipe 1: Frame Capture and Grayscale Conversion

Main functions:

- process_qr_frame
- rgb565_to_gray

What enters this pipe:

1. Camera frame buffer from esp_camera_fb_get.

What happens:

1. Camera access is protected by g_cam_mutex.
2. RGB565 is converted to grayscale luminance.
3. Frame is returned immediately to camera driver.

What leaves this pipe:

1. Grayscale frame in g_gray_buf.

Why this matters:

1. It prevents camera ownership conflicts with /capture and /stream handlers.

### Pipe 2: Preprocessing Buffers

Main functions:

- downsample_gray
- contrast_stretch
- adaptive_binarize
- adaptive_binarize_with_bias

What happens in current code path:

1. downsample_gray writes g_proc_raw_buf.
2. contrast_stretch writes g_proc_gray_buf.
3. Main quirc pass starts from raw grayscale (g_proc_raw_buf).
4. Adaptive main pass is optional and controlled by QR_MAIN_USE_ADAPTIVE (currently 0).

Current constants in code:

- QR_DOWNSCALE = 1
- QR_USE_ADAPTIVE_BINARIZE = 1
- QR_MAIN_USE_ADAPTIVE = 0

### Pipe 3: Multi-pass quirc Scanning and Decode Recovery

Main functions:

- run_quirc_scan
- decode_quirc_payload

Exact pass behavior (current implementation):

1. Pass A: main scan on primary buffer.
2. Pass B: retry on contrast-stretched grayscale if retries are enabled.
3. Pass C: raw-downsample retry only when decode streak logic allows deeper recovery.
4. Pass D: inverted grayscale retry every 8 decode-fail streak steps.
5. Pass E: adaptive-bias recovery sweep when decoded_ok is still 0.

Retry scheduling details:

1. Retry passes always run when count == 0.
2. If count > 0 but decoded_ok == 0, retries run every 4th decode-fail streak step.
3. Inverted pass runs every 8th decode-fail streak step.

Selection rule:

1. Prefer result set with higher decoded_ok.
2. If still no decode, may prefer higher raw count.

Telemetry support:

1. decode_err and decode_err_flip are exposed in /status.

### Pipe 4: Pose Estimation and Geometry Stabilization

Main functions:

- reorder_corners_tl_tr_br_bl
- compute_homography
- decompose_homography
- rotation_to_euler
- compute_qr_pose

What happens:

1. Corners are reordered to canonical TL, TR, BR, BL.
2. Planar homography is computed from known QR size.
3. Homography is decomposed to rotation + translation.
4. Rotation matrix is converted to roll, pitch, yaw.

Output units:

1. tx, ty, tz in millimeters.
2. roll, pitch, yaw in degrees.

### Pipe 5: Observation Probability, Gating, and Tracker

Main functions:

- detection_probability
- track_from_observation
- track_predict_only
- blend_angle_deg
- wrap_angle_deg

What happens:

1. Each candidate gets probability score p based on pose validity, decode flag, area, and continuity.
2. Best decoded candidate is preferred over slightly higher non-decoded candidate.
3. Observation is accepted if p >= accept threshold.
4. Decoded observations use relaxed acceptance threshold (accept_prob multiplied by 0.45).
5. If not accepted, tracker runs predict-only and decays confidence exponentially.

Current tracking constants:

- TRACK_MAX_HOLD_MS = 3500
- TRACK_DECAY_TAU_MS = 2600
- TRACK_MIN_CONF = 0.10
- OBS_ACCEPT_PROB = 0.15

### Pipe 6: HTTP Publication Layer

Main functions:

- data_handler
- status_handler
- capture_handler
- stream_handler

Endpoint map:

1. Port 80: /data, /status, /capture
2. Port 81: /stream

/data payload focus:

1. Global frame fields: frame_id, processing_ms, qr_fps, raw_count, decoded_count.
2. qr_codes[] entries: text, decoded, corners, estimated, confidence, age_ms, pose_valid, tx, ty, tz, roll, pitch, yaw.

/status payload focus:

1. Resource state: free_heap, psram_free, camera.
2. QR telemetry: qr_processing_ms, qr_fps, qr_raw, qr_decoded, qr_detections.
3. Decode debug: decode_err, decode_err_flip.
4. Tracker health: track_active, track_conf, track_age_ms.

### Pipe 7: PC Viewer and Logger

Main file: pc_client/vision_processor.py

Core components:

1. DataPoller thread continuously polls /data.
2. ESP32CamSource reads either /capture or /stream.
3. draw_overlay paints corners, lock box, confidence, age, and pose HUD.
4. format_qr_log prints compact terminal lines with decoded/estimated/pose fields.

Mode guidance:

1. Capture mode is default and usually better for robustness.
2. Stream mode gives smoother display but increases ESP-side load.

### Pipe 8: Scenario Logging and KPI Evaluation

Main files:

- scripts/run_scenario_overlay.ps1
- scripts/run_scenario_full.ps1
- scripts/run_fast_round.ps1
- pc_client/evaluate_session.py

Flow:

1. Save scenario metadata to scenario_log.csv.
2. Run overlay session.
3. Evaluate /data and /status over a configured duration.
4. Append one metrics row to scenario_metrics.csv.

Primary metrics:

1. detection_ratio
2. decode_ratio_when_detected
3. camera_status_ok_ratio
4. confidence_mean
5. tz_mean_mm and tz_std_mm
6. proc_ms_mean and proc_ms_p95

### Why Studying Functions Improves Robustness and Speed

Yes, this project can gain clear robustness and speed improvements by studying each function carefully.

Practical examples:

1. Preprocessing functions control how often quirc sees clean modules under difficult lighting.
2. Retry scheduling logic controls how much decode recovery you get versus how much latency you add.
3. Pose and corner-order functions control flip stability and angle jitter.
4. Tracker functions control dropout continuity versus stale estimates.
5. HTTP and viewer polling rates control system load and apparent responsiveness.

Beginner study order for optimization:

1. process_qr_frame
2. run_quirc_scan and decode_quirc_payload
3. compute_qr_pose path (homography and decomposition)
4. detection_probability and tracker functions
5. data_handler/status_handler plus PC poll intervals

## Hardware

| Item | Value |
|------|-------|
| Board | ESP32-S3 Cam |
| Camera | GC2145 (RGB565 path) |
| QR Size | 50 mm (default in firmware math) |
| USB-Serial | CH340 |

## Repository Layout

- firmware/cam_stream/src/main.cpp: on-device pipeline, tracker, HTTP API
- pc_client/vision_processor.py: laptop overlay + terminal logger
- pc_client/evaluate_session.py: session KPI evaluator
- scripts/run_scenario_full.ps1: scenario overlay + evaluator + CSV metrics append
- docs/study/qr_pose_math_study.ipynb: study notebook for QR pose math

## Quick Start

### 1. Flash firmware

```powershell
cd firmware/cam_stream
pio run --target upload
```

### 2. Install Python dependencies

```powershell
cd pc_client
py -3.12 -m pip install -r requirements.txt
```

### 3. Run viewer (capture mode default)

```powershell
cd pc_client
py -3.12 vision_processor.py --url http://192.168.1.2 --log-interval 0.25
```

### 4. Or use helper scripts

```powershell
# upload only
powershell -ExecutionPolicy Bypass -File scripts/run_firmware_upload.ps1

# viewer only
powershell -ExecutionPolicy Bypass -File scripts/run_viewer_overlay.ps1 -EspUrl http://192.168.1.2

# terminal logger only
powershell -ExecutionPolicy Bypass -File scripts/run_logger_only.ps1 -EspUrl http://192.168.1.2

# full scenario run with metrics append
powershell -ExecutionPolicy Bypass -File scripts/run_scenario_full.ps1 -EspUrl http://192.168.1.2
```

## API Quick Reference

| Endpoint | Port | Role |
|----------|------|------|
| /data | 80 | Primary machine-readable output for detections, pose, tracker state |
| /status | 80 | Health and telemetry summary |
| /capture | 80 | Single JPEG snapshot |
| /stream | 81 | MJPEG stream for overlay visualization |

## Tuning Guide (Latency vs Robustness)

| Knob | Increase For | Decrease For | Typical Side Effect |
|------|--------------|--------------|---------------------|
| QR_DOWNSCALE | lower CPU and faster loops | finer detail for small/far QR | too high can miss detail |
| QR_USE_ADAPTIVE_BINARIZE | better uneven-light recovery | less compute overhead | adaptive passes add time |
| TRACK_MAX_HOLD_MS | longer continuity through dropouts | faster stale-track expiry | long hold can keep old pose |
| TRACK_DECAY_TAU_MS | smoother confidence decay | more aggressive confidence drop | too small causes frequent invalidation |
| TRACK_MIN_CONF | stricter estimate acceptance | more persistent fallback | too low allows weak estimates |
| OBS_ACCEPT_PROB | stronger observation quality gate | faster tracker refresh | too low can inject jitter |

Recommended order:

1. Tune detection/decode first with lighting + preprocessing knobs.
2. Tune tracker hold/decay/gate second for continuity quality.
3. Tune latency last with downscale and stream usage.

## Data Contract for IK Integration

Recommended fixed vector:

```text
pose_ik = [
  tx_mm,
  ty_mm,
  tz_mm,
  roll_deg,
  pitch_deg,
  yaw_deg,
  confidence,
  decoded_flag,
  estimated_flag
]
```

## Study Resources

- Primary notebook: docs/study/qr_pose_math_study.ipynb
- Notebook-only documentation policy: all theory and integration notes are consolidated in the study notebook above.


## Troubleshooting

| Symptom | Typical Cause | Action |
|---------|---------------|--------|
| qr_fps near 0 | QR task not running or blocked | reboot, check serial logs, reflash |
| detections but decode fails | ECC failure due to blur/light/angle | improve lighting and focus, tune retry/preprocess knobs |
| viewer stutter/timeouts | network jitter or heavy stream load | prefer capture mode, reduce stream use |
| unstable estimated output | tracker thresholds too permissive | raise OBS_ACCEPT_PROB or TRACK_MIN_CONF |
| upload fails intermittently | serial port contention | close monitor/viewers and retry upload |
