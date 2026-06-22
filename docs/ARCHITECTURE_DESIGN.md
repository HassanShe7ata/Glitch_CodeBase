# Glitch Robot - System Architecture & Design Choices

## Academic/Industrial Documentation

**Version:** 2.0  
**Date:** 2026  
**Project:** Glitch - Omnidirectional Mecanum Wheel Robot with Computer Vision

---

## Table of Contents

1. [System Architecture](#system-architecture)
2. [Design Philosophy](#design-philosophy)
3. [Hardware Architecture](#hardware-architecture)
4. [Software Architecture](#software-architecture)
5. [Design Trade-offs](#design-trade-offs)
6. [Interesting Implementations](#interesting-implementations)

---

## System Architecture

### High-Level Architecture

The Glitch robot implements a **distributed edge computing architecture** with three independent computational nodes communicating via wireless protocols.

```
                    ┌─────────────────────────────────┐
                    │      Human-Machine Interface      │
                    │   (Phone/PC Web Browser)        │
                    └──────────────┬──────────────────┘
                                   │ WebSocket (ws://)
                                   ▼
┌─────────────────────────────────────────────────────────────────┐
│                        BASE NODE (ESP32)                    │
│  ┌─────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │ WiFi Access   │  │ AsyncWeb     │  │ ESP-NOW Hub  │  │
│  │ Point (AP)   │  │ Server       │  │ (Coordinator) │  │
│  │ Channel 11    │  │ Port 80      │  │               │  │
│  └─────────────┘  └──────────────┘  └───────┬──────┘  │
│                                            │             │
│  ┌─────────────────────────┐                │             │
│  │ Motor Control (I2C)     │                │             │
│  │ - Mecanum mixing         │                │             │
│  │ - KP position control    │                │             │
│  │ - Encoder feedback       │                │             │
│  └─────────────────────────┘                │             │
└───────────────────────────────────────────────┼─────────────┘
                                                │
                    ┌───────────────────────────┼─────────┐
                    │                           │         │
                    ▼                           ▼         │
        ┌───────────────────┐     ┌──────────────────┐   │
        │   ARM NODE (ESP32) │     │ CAMERA NODE      │   │
        │                     │     │ (ESP32-S3)       │   │
        │ - Inverse kinematics│     │                   │   │
        │ - Servo control    │     │ - QR detection    │   │
        │ - Trajectory execute│     │ - Pose estimation │   │
        │ - Status reporting  │     │ - HTTP streaming  │   │
        └───────────────────┘     └──────────────────┘   │
                    │                           │         │
                    └───────────────────────────┼─────────┘
                                                │
                                        ESP-NOW (250KB/s)
```

---

### Node Responsibilities

#### 1. Base Node (Coordinator)

**Role:** Central hub for communication and low-level motor control

**Responsibilities:**
- WiFi Access Point (AP) for phone/PC connection
- WebSocket server for real-time dashboard
- ESP-NOW coordinator (routes packets between nodes)
- I2C master for motor driver communication
- Mecanum wheel mixing and KP position control
- Autonomous mode coordination

**Design Choice:** Single coordinator avoids communication conflicts and provides deterministic routing.

---

#### 2. Arm Node (Executor)

**Role:** Dexterous manipulator with 5-DOF inverse kinematics

**Responsibilities:**
- Receive commands via ESP-NOW
- Calculate inverse kinematics (IK) for target poses
- Execute synchronized servo movements
- Report busy/idle status to Base
- Receive camera pose data for guided pickup

**Design Choice:** Command queue decouples WiFi callbacks from servo execution (prevents blocking).

---

#### 3. Camera Node (Sensor)

**Role:** Computer vision processing with on-device QR detection

**Responsibilities:**
- Capture image frames (GC2145 sensor)
- Detect and decode QR codes
- Estimate 3D pose (position + orientation)
- Temporal tracking across frames
- Stream MJPEG video (optional)

**Design Choice:** On-device processing reduces bandwidth and latency vs. streaming raw images.

---

## Design Philosophy

### 1. Distributed Edge Computing

**Principle:** Each node performs local computation instead of centralizing all processing.

**Benefits:**
- **Fault Isolation:** Node failure doesn't crash entire system
- **Scalability:** Add nodes without rewriting central controller
- **Real-time Performance:** No single bottleneck

**Example:**
- Camera: QR detection + pose estimation (computationally expensive)
- Arm: Inverse kinematics solver (floating-point math)
- Base: Simple command routing + motor control

---

### 2. Hybrid Communication Protocols

**Principle:** Use the right protocol for each communication pattern.

| Pattern | Protocol | Rationale |
|---------|----------|------------|
| **Human ↔ Machine** | WebSocket | Bi-directional, low latency |
| **Node ↔ Node** | ESP-NOW | Deterministic, no TCP overhead |
| **Actuator Control** | I2C | Synchronous, ACK-based |
| **Configuration** | HTTP | Standard, browser-compatible |

**Design Choice:** Avoid "one protocol to rule them all" — each layer has different requirements.

---

### 3. Asynchronous Event-Driven Architecture

**Principle:** Avoid blocking operations; use callbacks and queues.

**Implementation:**
```c
// WRONG: Blocking in callback
void OnDataRecv(uint8_t* mac, uint8_t* data, int len) {
    processData(data);  // May take 100ms → WiFi task blocked
}

// CORRECT: Enqueue for later processing
void OnDataRecv(uint8_t* mac, uint8_t* data, int len) {
    enqueueCmd(data);  // < 1ms
}

void loop() {
    if (dequeueCmd(cmd)) {
        processCmd(cmd);  // Execute in loop task
    }
}
```

**Benefits:**
- WiFi callbacks return quickly (ESP-NOW reliability)
- Servo movements can yield to WiFi task
- No priority inversion

---

### 4. Fail-Safe and Degraded Modes

**Principle:** System should continue operating even if some components fail.

**Examples:**
1. **ESP-NOW init fails → Web server still starts** (Issue #1 from debugging)
2. **Camera offline → Manual arm control still works**
3. **I2C error → Abort movement, report error**
4. **Autonomous mode timeout → Return to manual control**

**Design Choice:** Graceful degradation instead of catastrophic failure.

---

## Hardware Architecture

### Computing Nodes

#### Base Node: ESP32-WROOM-32

**Specifications:**
- **CPU:** Dual-core Xtensa LX6 @ 240 MHz
- **RAM:** 520 KB SRAM
- **Flash:** 4 MB
- **WiFi:** 802.11 b/g/n
- **I2C:** Master mode, 40 kHz

** why This Chip?**
- Dual-core enables task isolation (WiFi vs. motor control)
- Sufficient RAM for WebSocket buffers
- Native ESP-NOW support

---

#### Arm Node: ESP32-WROOM-32

**Specifications:** Same as Base Node

**Why Same Chip as Base?**
- ESP-NOW compatibility (same WiFi stack)
- Cost optimization (volume pricing)
- Code reuse (shared ESP-NOW structs)

---

#### Camera Node: ESP32-S3-WROOM-1

**Specifications:**
- **CPU:** Xtensa LX7 @ 240 MHz (faster than ESP32)
- **RAM:** 512 KB SRAM + 8 MB PSRAM
- **Flash:** 16 MB
- **Camera Interface:** DVP (Digital Video Port)
- **WiFi:** 802.11 b/g/n (compatible with ESP32)

**Why ESP32-S3 (Not ESP32)?**
- **PSRAM:** QR detection buffers require ~1 MB RAM (ESP32 only has 520 KB)
- **DVP Camera Interface:** ESP32-S3 has dedicated hardware (ESP32 uses SPI camera)
- **Faster CPU:** QR processing is computationally intensive

---

### Motor Driver: I2C Brushed DC Motor Controller

**Interface:** I2C (Address: 0x34)  
**Capabilities:**
- 4x DC motor channels (mecanum wheels)
- 16-bit encoder counters
- PID speed control (internal to driver)

**Why I2C Instead of PWM?**
- **Encoder Feedback:** Driver maintains encoder counts (reduces ESP32 interrupt load)
- **Simplified Wiring:** 2 wires for 4 motors + encoders
- **Deterministic Timing:** I2C clock ensures synchronized updates

---

### Servo Driver: PCA9685 (Arm Node)

**Interface:** I2C (Address: 0x40)  
**Capabilities:**
- 16-channel PWM servo driver
- 12-bit resolution (0-4095)
- 50 Hz update rate (20ms period)

**Why PCA9685 Instead of ESP32 PWM?**
- **Independent Servo Power:** Separate 5V rail for servos
- **More Channels:** ESP32 only has 16 PWM channels (need 5 for arm + future expansion)
- **I2C Multiplexing:** Can add more PCA9685 chips on same bus

---

### Camera Module: GC2145 (ESP32-S3)

**Specifications:**
- **Resolution:** 2 MP (1600x1200 max)
- **Interface:** DVP (8-bit parallel)
- **Frame Rate:** 30 FPS @ 640x480
- **Lens:** 62° FOV (matches `CAMERA_FOV_DEG`)

**Why This Camera?**
- **Cost:** < $10 USD
- **ESP32-S3 Compatibility:** Native DVP driver in ESP-IDF
- **Sufficient Resolution:** 640x480 adequate for QR detection at 2m distance

---

## Software Architecture

### Base Node Software Stack

```
┌─────────────────────────────────────┐
│   Application Layer                 │
│   - Autonomous mode state machine   │
│   - Telemetry aggregation           │
│   - Command routing                │
└──────────────┬────────────────────┘
               │
┌──────────────▼────────────────────┐
│   Communication Layer               │
│   - AsyncWebServer (WebSocket)    │
│   - ESP-NOW (peer management)    │
│   - I2C (motor driver)           │
└──────────────┬────────────────────┘
               │
┌──────────────▼────────────────────┐
│   Control Layer                     │
│   - Mecanum wheel mixing          │
│   - KP position control            │
│   - Encoder feedback               │
└──────────────┬────────────────────┘
               │
┌──────────────▼────────────────────┐
│   Hardware Abstraction Layer (HAL) │
│   - WiFi (AP mode)                │
│   - I2C driver                    │
│   - Encoder counters              │
└────────────────────────────────────┘
```

---

### Arm Node Software Stack

```
┌─────────────────────────────────────┐
│   Application Layer                 │
│   - Pick-and-place sequences       │
│   - Color-based sorting            │
│   - Camera-guided pickup           │
└──────────────┬────────────────────┘
               │
┌──────────────▼────────────────────┐
│   Motion Planning Layer             │
│   - Inverse kinematics solver      │
│   - Forward kinematics             │
│   - Trajectory interpolation       │
└──────────────┬────────────────────┘
               │
┌──────────────▼────────────────────┐
│   Communication Layer               │
│   - ESP-NOW receive callback      │
│   - Command queue (ring buffer)   │
│   - Status reporting               │
└──────────────┬────────────────────┘
               │
┌──────────────▼────────────────────┐
│   Actuation Layer                   │
│   - PCA9685 servo driver          │
│   - Synchronized PWM updates      │
│   - Smooth interpolation           │
└────────────────────────────────────┘
```

---

### Camera Node Software Stack

```
┌─────────────────────────────────────┐
│   Application Layer                 │
│   - Temporal QR tracking           │
│   - Confidence estimation          │
│   - Color classification          │
└──────────────┬────────────────────┘
               │
┌──────────────▼────────────────────┐
│   Computer Vision Layer             │
│   - Multi-pass QR detection       │
│   - Pose estimation (homography)  │
│   - Adaptive binarization          │
└──────────────┬────────────────────┘
               │
┌──────────────▼────────────────────┐
│   Image Processing Layer            │
│   - RGB565 to grayscale           │
│   - Contrast stretching           │
│   - Downsampling                  │
└──────────────┬────────────────────┘
               │
┌──────────────▼────────────────────┐
│   Communication Layer               │
│   - ESP-NOW send                  │
│   - HTTP server (optional)        │
│   - MJPEG streaming               │
└──────────────┬────────────────────┘
               │
┌──────────────▼────────────────────┐
│   Hardware Abstraction Layer (HAL) │
│   - DVP camera interface          │
│   - PSRAM frame buffers          │
│   - ESP32-S3 WiFi stack          │
└────────────────────────────────────┘
```

---

## Design Trade-offs

### 1. ESP-NOW vs. WiFi/TCP

| Criteria | ESP-NOW | WiFi/TCP |
|----------|----------|----------|
| **Latency** | 2-5 ms | 10-50 ms |
| **Payload** | 250 bytes | 1460 bytes (MTU) |
| **Reliability** | Best-effort | Guaranteed delivery |
| **Encryption** | Optional | Built-in (WPA2) |
| **Topology** | Many-to-many | Point-to-point |

**Decision:** ESP-NOW for node-to-node, WebSocket for human interface.

---

### 2. On-Device vs. Server-Side Vision

| Criteria | On-Device (Camera) | Server-Side (Base) |
|----------|----------------------|---------------------|
| **Latency** | < 200 ms | 500+ ms (image transfer) |
| **Bandwidth** | 23 bytes (pose only) | ~100 KB (compressed image) |
| **Complexity** | ESP32-S3 + PSRAM | Base ESP32 insufficient |
| **Privacy** | Image never leaves device | Image transmitted wirelessly |

**Decision:** On-device processing with ESP32-S3 (requires PSRAM).

---

### 3. Synchronous vs. Asynchronous IK

| Criteria | Synchronous | Asynchronous |
|----------|--------------|---------------|
| **Code Complexity** | Simple | Complex (state machine) |
| **Responsiveness** | Blocked during IK | Can process WebSocket msgs |
| **Determinism** | Guaranteed sequence | May interleave with other tasks |

**Decision:** Synchronous IK with periodic `delay(10)` yields (allows WiFi callbacks).

---

### 4. Text vs. Binary Commands (Arm Node)

| Criteria | Text (`"H"`, `"GTC"`) | Binary (struct) |
|----------|------------------------|------------------|
| **Debugging** | Human-readable | Requires hex decoder |
| **Flexibility** | Easy to add commands | Struct size fixed |
| **Parsing** | `strcmp()` overhead | `memcpy()` only |
| **Error Handling** | Typos cause silent failures | Invalid binary rejected |

**Decision:** Text for simple commands, binary structs for pose data.

---

### 5. PROGMEM for Web Dashboard

**Question:** Should HTML be stored in Flash (PROGMEM) or served from SPIFFS?

| Criteria | PROGMEM | SPIFFS |
|----------|----------|---------|
| **Flash Usage** | Uses program space | Uses filesystem partition |
| **Updates** | Re-flash required | Over-the-air (OTA) possible |
| **Performance** | Fast (memory-mapped) | Slower (filesystem read) |
| **Complexity** | Simple (single `.ino`) | Requires partition table |

**Decision:** PROGMEM for simplicity (dashboard is small: ~15 KB).

---

## Interesting Implementations

### 1. Multi-Pass QR Detection (Camera Node)

**Problem:** Single-pass QR detection fails under challenging lighting.

**Solution:** Four-pass detection with fallback strategies.

```c
// Pass 1: Raw grayscale (fastest)
run_quirc_scan(g_proc_buf, dets, ...);

// Pass 2: Contrast-stretched (handles low contrast)
if (decoded_ok == 0) {
    run_quirc_scan(g_proc_gray_buf, dets, ...);
}

// Pass 3: Inverted grayscale (handles negative QR)
if (decoded_ok == 0) {
    for (int i = 0; i < n; i++) {
        g_proc_buf[i] = 255 - g_proc_gray_buf[i];  // Invert
    }
    run_quirc_scan(g_proc_buf, dets, ...);
}

// Pass 4: Adaptive binarization with bias sweep
if (decoded_ok == 0) {
    for (int bias : {2, 3, 4, 5}) {
        adaptive_binarize_with_bias(g_proc_buf, w, h, bias);
        run_quirc_scan(g_proc_buf, dets, ...);
    }
}
```

**Why Interesting?**
- **Robustness:** Handles diverse lighting conditions
- **Performance:** Early exit when QR decoded (avoids unnecessary passes)
- **Configurable:** `QR_USE_ADAPTIVE_BINARIZE` compile-time flag

---

### 2. Temporal Tracking with Exponential Decay

**Problem:** QR codes may be temporarily occluded (robot arm passes in front).

**Solution:** Kalman-filter-inspired temporal tracking.

```c
static bool track_predict_only(uint32_t now_ms) {
    uint32_t age = now_ms - g_track.last_obs_ms;
    
    // Exponential decay: confidence drops over time
    float conf = expf(-((float)age) / TRACK_DECAY_TAU_MS);
    
    g_track.confidence = conf;
    g_track.det.estimated = true;  // Mark as prediction
    
    // Give up after max hold time or min confidence
    if (age > TRACK_MAX_HOLD_MS || conf < TRACK_MIN_CONF) {
        g_track.active = false;
        return false;
    }
    return true;
}
```

**Why Interesting?**
- **Bio-inspired:** Mimics human visual persistence
- **Tunable:** `TRACK_DECAY_TAU_MS` and `TRACK_MAX_HOLD_MS` adjust robustness
- **Probabilistic:** Confidence score enables sensor fusion

---

### 3. Mecanum Wheel Mixing Matrix

**Problem:** Convert (forward, strafe, rotate) to (M1, M2, M3, M4) speeds.

**Solution:** Linear mixing derived from wheel geometry.

```
Forward:    [ 1, -1, -1,  1]
Backward:   [-1,  1,  1, -1]
Strafe R:   [ 1,  1,  1,  1]
Strafe L:   [-1, -1, -1, -1]
Rotate CW:   [ 1, -1,  1, -1]
Rotate CCW:  [-1,  1, -1,  1]
```

**Derivation:**
- Each wheel contributes to (forward, strafe, rotation) based on its position
- Sign determines direction of rotation
- Mixing matrix is the pseudoinverse of the wheel Jacobian

**Why Interesting?**
- **Omnidirectional:** Robot can move in any direction without turning
- **Simple Math:** No iterative solvers (just addition/multiplication)
- **Generalizable:** Works for any mecanum configuration (3-wheel, 6-wheel)

---

### 4. Inverse Kinematics for 5-DOF Arm

**Problem:** Given end-effector position (x, y, z) and pitch angle (φ), find joint angles (θ₁, θ₂, θ₃, θ₄).

**Solution:** Geometric approach (closed-form solution).

```c
// Step 1: Calculate base rotation (θ₁)
angles.t1 = atan2(y, x);

// Step 2: Project to 2D plane (shoulder/elbow)
float R = sqrt(x² + y²) - L5*cos(φ);  // Wrist offset
float Z = z - L1 - L5*sin(φ);

// Step 3: Solve 2-link planar arm (elbow and wrist)
float cos_q3 = (R² + Z² - L2² - L3² - L4²) / (2 * L4 * sqrt(L2² + L3²));
float q3 = acos(cos_q3);  // Elbow angle
float q2 = atan2(Z, R) + atan2(L4*sin(q3), L2 + L3 + L4*cos(q3));  // Shoulder angle

// Step 4: Calculate wrist pitch (θ₄)
angles.t4 = 90 - (φ - q2 + q3) * 180/π;
```

**Why Interesting?**
- **Closed-Form:** No numerical iteration (fast: < 1ms)
- **Multiple Solutions:** Elbow-up vs. elbow-down (selected by `acos()`)
- **Workspace Limits:** Constrained by link lengths (L1-L5)

---

### 5. Camera-to-Arm Coordinate Transform

**Problem:** Camera detects QR in camera frame, but arm needs target in arm base frame.

**Solution:** 4x4 homogeneous transform matrix.

```c
static const float T_CAM_TO_L4[4][4] = {
    { 0.0f,  0.0f,  1.0f,  -5.9f    },  // Camera X → Arm Z
    { 0.0f, -1.0f,  0.0f,   6.35f   },  // Camera Y → -Arm Y
    { 1.0f,  0.0f,  0.0f,   0.0f    },  // Camera Z → Arm X
    { 0.0f,  0.0f,  0.0f,   1.0f    }
};
```

**Transformation Pipeline:**
1. **Camera Frame:** QR position (tx, ty, tz) from pose estimation
2. **Link4 Frame:** Transform by `T_CAM_TO_L4`
3. **Arm Base Frame:** Add forward kinematics offset

**Why Interesting?**
- **Extrinsic Calibration:** Matrix `T_CAM_TO_L4` must be measured experimentally
- **Frame Confusion:** Camera Z is depth, Arm Z is height (requires careful sign handling)
- **Robustness:** Small calibration errors cause large end-effector position errors

---

### 6. ESP-NOW Callback ABI Mismatch (Bug & Fix)

**Problem:** ESP-IDF 5.x changed callback signature, but Arduino wrapper still shows old signature.

**Root Cause:** Application Binary Interface (ABI) mismatch.

```c
// OLD (ESP-IDF 4.x / Arduino-ESP32 2.x)
typedef void (*esp_now_recv_cb_t)(const uint8_t *mac_addr, 
                                   const uint8_t *data, 
                                   int data_len);

// NEW (ESP-IDF 5.x / Arduino-ESP32 3.x)
typedef void (*esp_now_recv_cb_t)(const esp_now_recv_info_t *esp_now_info,
                                   const uint8_t *data,
                                   int data_len);
```

**Why Compiles But Doesn't Work:**
- C++ allows function pointer casting (`-fpermissive`)
- WiFi task calls function with `esp_now_recv_info_t*`
- Callback expects `uint8_t*` → interprets struct pointer as MAC address → undefined behavior

**Fix:**
```c
void OnDataRecv(const esp_now_recv_info_t *info, 
                const uint8_t *data, 
                int len) {
    const uint8_t *mac_addr = info->src_addr;  // Extract MAC
    // ... rest of processing
}
```

**Why Interesting?**
- **Silent Failure:** No compiler warning, no runtime error (just doesn't work)
- **Espressif Documentation Lag:** Official docs not updated for signature change
- **Community Debugging:** Required reading GitHub issues and StackOverflow

---

## Performance Analysis

### Computational Load

| Node | Task | CPU Usage | RAM Usage |
|------|------|-----------|-----------|
| **Base** | WebSocket server | ~5% | ~20 KB |
| **Base** | ESP-NOW routing | ~2% | ~1 KB |
| **Base** | Motor control (KP) | ~10% | ~2 KB |
| **Arm** | Inverse kinematics | < 1% | ~5 KB |
| **Arm** | Servo interpolation | ~15% | ~10 KB |
| **Camera** | QR detection (quirc) | ~40% | ~800 KB (PSRAM) |
| **Camera** | HTTP streaming | ~20% | ~100 KB |

---

### Communication Bandwidth

| Interface | Data Rate | Typical Payload | Frequency |
|-----------|--------------|------------------|-----------|
| **ESP-NOW (Base→Arm)** | ~100 kbps | 23 bytes | On demand |
| **ESP-NOW (Camera→Base)** | ~50 kbps | 23 bytes | 5 Hz |
| **ESP-NOW (Arm→Base)** | ~10 kbps | 4 bytes | 5 Hz |
| **WebSocket (Base→Phone)** | ~50 kbps | ~200 bytes | 2 Hz |
| **I2C (Base→Motor)** | ~2.5 kbps | 16 bytes | 50 Hz |

---

### Battery Life Estimation

**Assumptions:**
- Base node: 2A @ 5V (10W)
- Arm node: 1A @ 5V (5W)
- Camera node: 1.5A @ 5V (7.5W)
- Battery: 3S LiPo (11.1V, 5000 mAh = 55.5 Wh)

**Runtime:**
```
Total power: 10W + 5W + 7.5W = 22.5W
Runtime: 55.5 Wh / 22.5W = ~2.5 hours
```

**Optimization Opportunities:**
- Camera: Disable HTTP server (saves ~2W)
- Arm: Reduce servo update rate (saves ~1W)
- Base: WiFi power save mode (saves ~1W)

---

## Scalability Considerations

### Adding More Nodes

**Current Limit:** ESP-NOW supports up to **20 peers** per device.

**Expansion Scenarios:**

1. **Additional Sensors** (e.g., Lidar, IMU)
   - Add as ESP-NOW peers
   - Base routes data to phone via WebSocket

2. **Swarm Robotics** (multiple Glitch robots)
   - Each robot = 1 Base + 1 Arm + 1 Camera
   - Inter-robot communication via ESP-NOW broadcast

3. **Cloud Connectivity**
   - Add Raspberry Pi as WiFi station (connects to Base AP)
   - Pi uploads telemetry to cloud (MQTT/WebSocket)

---

### Increasing Camera Resolution

**Current:** 640x480 (VGA) @ 4 FPS

**Upgrade Path:**
- **ESP32-S3:** Increase to 800x600 (SVGA) @ 2 FPS
- **External ISP:** Add dedicated vision processor (e.g., K210) via SPI

**Trade-off:** Higher resolution → slower FPS → reduced responsiveness

---

## Conclusion

The Glitch robot demonstrates a **practical distributed robotics architecture** balancing performance, cost, and developability.

**Key Design Wins:**
1. **ESP-NOW** for low-latency inter-node communication
2. **On-device vision** for bandwidth efficiency
3. **Asynchronous architecture** for responsiveness
4. **Graceful degradation** for robustness

**Areas for Future Improvement:**
1. **Encryption** for ESP-NOW (production security)
2. **Sensor fusion** (IMU + camera for pose estimation)
3. **OTA updates** (over-the-air firmware updates)
4. **ROS2 integration** (for research applications)

---

## References

1. **Espressif ESP-NOW Docs:** https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html
2. **Mecanum Wheel Kinematics:** https://www.cs.cmu.edu/~rasc/Download/AMRobots3/03-Buehler-IntroMecanum.ppt.pdf
3. **Inverse Kinematics Tutorial:** https://www.youtube.com/watch?v=8wtae6e2qY
4. **QR Code Detection (quirc):** https://github.com/dlbeer/quirc
5. **Homography Estimation:** https://docs.opencv.org/4.x/d9/dab/tutorial_homography.html

---

**End of Document**
