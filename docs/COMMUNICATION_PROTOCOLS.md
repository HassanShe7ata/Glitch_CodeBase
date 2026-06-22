# Glitch Robot - Communication Protocols Specification

## Academic/Industrial Documentation

**Version:** 2.0  
**Date:** 2026  
**Project:** Glitch - Omnidirectional Mecanum Wheel Robot with Computer Vision

---

## Table of Contents

1. [System Overview](#system-overview)
2. [ESP-NOW Protocol](#esp-now-protocol)
3. [WebSocket Protocol](#websocket-protocol)
4. [HTTP Endpoints](#http-endpoints)
5. [I2C Communication](#i2c-communication)
6. [Protocol Design Rationale](#protocol-design-rationale)

---

## System Overview

### Distributed Architecture

The Glitch robot employs a **distributed multi-node architecture** with three ESP32-based microcontroller nodes:

```
┌─────────────────┐         ┌─────────────────┐         ┌─────────────────┐
│   Base Node     │         │   Arm Node      │         │  Camera Node    │
│                 │         │                 │         │                 │
│ - WiFi AP       │◄───────►│ - Servo Control │         │ - QR Detection  │
│ - WebSocket Srv │ ESP-NOW│ - IK Solver     │◄───────►│ - Pose Estimation│
│ - Motor Control │         │ - I2C (PCA9685)│         │ - ESP32-S3      │
└─────────────────┘         └─────────────────┘         └─────────────────┘
         ▲                          ▲                          ▲
         │                          │                          │
         └──────────────────────────┴──────────────────────────┘
                           ESP-NOW Wireless Protocol
```

### Communication Layers

| Layer | Protocol | Purpose | Latency | Reliability |
|-------|----------|---------|--------|-------------|
| **Inter-Node** | ESP-NOW | Low-latency command/telemetry | < 5ms | Best-effort |
| **Human-Machine** | WebSocket | Real-time control interface | < 50ms | Acknowledged |
| **Configuration** | HTTP | Static file serving | N/A | TCP reliable |
| **Motor Control** | I2C | High-speed motor commands | < 1ms | ACK-based |

---

## ESP-NOW Protocol

### Overview

**ESP-NOW** is Espressif's connectionless communication protocol optimized for low power and low latency. It uses IEEE 802.11 WiFi frames at the link layer without association.

#### Design Choice: Why ESP-NOW?

1. **Deterministic Latency**: No TCP handshake, no IP routing
2. **Broadcast Capability**: One-to-many communication without acknowledgements
3. **Low Power**: No beacon transmission in sleep modes
4. **Payload Efficiency**: 250-byte MTU (sufficient for robotics commands)

### Packet Structure

All ESP-NOW packets use **packed structs** to ensure memory alignment across different ESP32 architectures (ESP32 vs ESP32-S3).

```c
// Attribute packed prevents compiler padding
struct __attribute__((packed)) PacketStruct {
    uint8_t type;       // Packet type identifier
    // ... payload fields
};
```

#### Why `__attribute__((packed))`?

- **Memory Alignment**: Different architectures may insert padding bytes
- **Wire Compatibility**: Ensures sender and receiver interpret bytes identically
- **Size Optimization**: Removes unused padding (critical for 250-byte limit)

---

### Packet Types

#### 1. ScanRequest (Base → Camera)

**Purpose:** Trigger QR code detection on Camera node

```c
struct __attribute__((packed)) ScanRequest {
    uint8_t type;        // 0x20 (ESPNOW_TYPE_SCAN_REQ)
    uint8_t task_id;     // Incremental task identifier
    uint8_t mode;        // 0=scan_qr, 1=scan_platform
    uint8_t reserved;    // Future use / alignment
};
```

**Size:** 4 bytes  
**Design Choice:** Task ID enables asynchronous request/reply matching

---

#### 2. PoseReply (Camera → Base)

**Purpose:** Return QR pose estimation to Base

```c
struct __attribute__((packed)) PoseReply {
    uint8_t type;        // 0x30 (ESPNOW_TYPE_POSE_REPLY)
    uint8_t task_id;     // Matches ScanRequest.task_id
    uint8_t pose_valid;  // 0=invalid, 1=valid
    uint8_t color;       // 0=unknown, 1=R, 2=G, 3=B
    uint8_t estimated;    // 0=measured, 1=prediction
    float tx_mm;         // X translation (mm)
    float ty_mm;         // Y translation (mm)
    float tz_mm;         // Z translation (mm)
    float yaw_deg;       // Yaw angle (degrees)
    float confidence;     // 0.0-1.0 confidence score
};
```

**Size:** 23 bytes  
**Key Features:**
- `estimated` flag indicates temporal prediction vs. direct measurement
- `confidence` enables sensor fusion with other detection methods
- Floating-point fields use IEEE 754 single-precision (4 bytes each)

---

#### 3. CameraPoseData (Base → Arm)

**Purpose:** Forward camera pose to Arm for inverse kinematics

```c
struct __attribute__((packed)) CameraPoseData {
    uint8_t type;        // 1 (camera pose packet)
    uint8_t pose_valid;
    uint8_t color;
    uint8_t estimated;
    float tx_mm;
    float ty_mm;
    float tz_mm;
    float yaw_deg;
    float confidence;
};
```

**Size:** 23 bytes  
**Design Choice:** Duplicate structure (not shared with PoseReply) to allow independent evolution

---

#### 4. ArmStatus (Arm → Base)

**Purpose:** Report Arm node busy/idle state

```c
struct __attribute__((packed)) ArmStatus {
    uint8_t type;   // 0 (status packet)
    uint8_t busy;   // 1=busy, 0=idle
    uint8_t pad[2]; // Explicit padding for alignment
};
```

**Size:** 4 bytes  
**Design Choice:** `pad[2]` ensures 4-byte alignment for potential DMA access

---

#### 5. ArmCommand (Base → Arm)

**Purpose:** Text-based command interface for Arm control

```c
typedef struct {
    char command[10];  // Null-terminated command string
} ArmCommand;
```

**Size:** 10 bytes  
**Supported Commands:**
- `"H"` - Home position
- `"S"` - Scan pose
- `"GTC"` - Green to Car
- `"BTC"` - Blue to Car
- `"RTC"` - Red to Car
- `"SV:i:a"` - Servo step (i=joint index, a=angle)

**Design Choice:** Text-based commands for human readability during debugging

---

### ESP-NOW Callback Architecture

#### Critical Implementation Detail: Callback Signature

**ESP-IDF 5.x (Arduino-ESP32 3.x) Changed the Callback Signature:**

```c
// OLD Signature (ESP-IDF 4.x / Arduino-ESP32 2.x)
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len)

// NEW Signature (ESP-IDF 5.x / Arduino-ESP32 3.x)
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
```

**ABI Mismatch Problem:**
- Old signature compiles with `-fpermissive` but **silently fails at runtime**
- WiFi task passes `esp_now_recv_info_t*` but function expects `uint8_t*`
- **Result:** Callback never fires, packets appear "lost"

**Correct Implementation:**
```c
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    const uint8_t *mac_addr = info->src_addr;  // Extract MAC from info struct
    // ... packet processing
}
```

> **FIX APPLIED (2026-06-17):** All three active firmware files (`base.cpp`, `arm/arm.cpp`, `camera/src/main.cpp`) have been updated from the old `const uint8_t*` signature to the correct `const esp_now_recv_info_t*` signature. Send callbacks also updated to `const esp_now_send_info_t*` where applicable.

---

### Peer Management

#### MAC Address Configuration

**Design Choice:** Static MAC address assignment for deterministic peer discovery

```c
// Base node MAC — AP mode (ESP-NOW peers must use AP MAC, not STA MAC)
// STA MAC (efuse): 80:F3:DA:42:3E:5C
// AP  MAC (STA+1): 80:F3:DA:42:3E:5D
uint8_t baseAddress[] = {0x80, 0xF3, 0xDA, 0x42, 0x3E, 0x5D};

// Arm node MAC (STA mode)
uint8_t armAddress[] = {0x68, 0xFE, 0x71, 0x12, 0x5D, 0xA8};

// Camera node MAC (STA mode)
uint8_t cameraAddress[] = {0x94, 0xA9, 0x90, 0x08, 0xB2, 0xB8};
```

**Why Static MACs?**
1. ** Deterministic Routing**: No dynamic address resolution needed
2. **Security**: MAC filtering can be implemented
3. **Debugging**: Serial logs show known device identities

#### Channel Configuration

**Critical Requirement:** All ESP-NOW peers **must** use the same WiFi channel

```c
const uint8_t WIFI_CHANNEL = 11;  // Base AP channel
```

**Design Choice:** Channel 11 (2.4GHz, least congested in typical environments)

---

### ESP-NOW Initialization Sequence

```c
// 1. Initialize ESP-NOW
if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW INIT FAILED");
    // Non-fatal: Continue without ESP-NOW (web server still works)
}

// 2. Register callbacks
esp_now_register_send_cb(OnDataSent);      // Optional: Track send status
esp_now_register_recv_cb(OnDataRecv);      // Required: Receive packets

// 3. Add peers
esp_now_peer_info_t peerInfo = {};
memcpy(peerInfo.peer_addr, peerAddress, 6);
peerInfo.channel = WIFI_CHANNEL;  // MUST match AP channel
peerInfo.encrypt = false;
peerInfo.ifidx = WIFI_IF_AP;    // For AP mode (Base)
// or WIFI_IF_STA for STA mode (Arm, Camera)

esp_now_add_peer(&peerInfo);
```

**Error Handling:** Peer addition failure is non-fatal (logs error, continues)

---

## WebSocket Protocol

### Overview

**WebSocket** provides full-duplex communication between the web dashboard (phone/PC) and the Base ESP32.

#### Design Choice: Why WebSocket?

1. **Low Latency**: Persistent connection (no HTTP overhead per message)
2. **Bi-directional**: Server can push telemetry without client polling
3. **Browser Native**: No plugins required
4. **JSON Compatible**: Human-readable protocol for debugging

---

### Message Format

All WebSocket messages use **JSON** format:

#### Client → Server (Commands)

```json
{
    "cmd": "MOVE",
    "arg": "FWD"
}
```

**Supported Commands:**

| `cmd` | `arg` | Purpose |
|--------|-------|---------|
| `MOVE` | `FWD`, `BACK`, `LEFT`, `RIGHT` | Mecanum drive |
| `MOVE` | `ROTCW`, `ROTCCW` | Rotation |
| `MOVE` | `DIAGFR`, `DIAGFL`, `DIAGBR`, `DIAGBL` | Diagonal movement |
| `MOVE` | `STOP` | Emergency stop |
| `STEP` | `FWD`, `BACK`, `LEFT`, `RIGHT` | Position-controlled move |
| `SPEED` | `0-100` | Motor speed setting |
| `ARM` | `H`, `S`, `GTC`, `BTC`, `RTC` | Arm commands |
| `ARM` | `CAM_PICKUP` | Camera-guided pickup |
| `SCAN` | `QR`, `PLAT` | Trigger camera scan |
| `SERVO` | `i:UP`, `i:DOWN` | Servo step control |

---

#### Server → Client (Telemetry)

```json
{
    "type": "telemetry",
    "confidence": 0.85,
    "yaw": 12.5,
    "color": "RED",
    "distance_mm": 250,
    "tx_mm": 120,
    "ty_mm": -30,
    "motor_speed": 25,
    "free_heap": 45000,
    "autonomous": false,
    "connected": true
}
```

---

#### Server → Client (QR Result)

```json
{
    "type": "qr_result",
    "pose_valid": 1,
    "color": 1,
    "color_name": "RED",
    "confidence": 0.92,
    "tx_mm": 125.0,
    "ty_mm": -28.0,
    "tz_mm": 245.0,
    "yaw_deg": 11.5,
    "estimated": 0
}
```

---

#### Server → Client (Arm Status)

```json
{
    "type": "arm_status",
    "busy": true
}
```

---

### WebSocket Event Handling

```c
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
    switch (type) {
        case WS_EVT_CONNECT:    // Client connected
        case WS_EVT_DISCONNECT: // Client disconnected
        case WS_EVT_DATA:       // Data received
            handleWebSocketMessage(arg, data, len);
        case WS_EVT_PONG:       // Keep-alive pong
        case WS_EVT_ERROR:      // Error occurred
    }
}
```

**Design Choice:** Non-blocking event-driven architecture (no `while` loops in callbacks)

---

## HTTP Endpoints

### Base Node (Port 80)

| Endpoint | Method | Purpose | Response |
|----------|--------|---------|----------|
| `/` | GET | Serve web dashboard (controller HTML) | HTML (PROGMEM) |
| `/status` | GET | Robot status (arm busy, QR results, telemetry) | JSON |
| `/cmd` | POST | Receive commands from dashboard (move, arm, scan) | JSON `{"ok":true}` |

> **INCONSISTENCY**: Old docs listed `/healthz` — this endpoint does not exist in current `base.cpp`. Replaced by `/status` and `/cmd`.

---

### Camera Node (Port 80)

| Endpoint | Method | Purpose | Response |
|----------|--------|---------|----------|
| `/capture` | GET | Single JPEG snapshot | `image/jpeg` |
| `/data` | GET | QR detection results | JSON |
| `/status` | GET | Camera health/telemetry | JSON |

> **INCONSISTENCY**: Camera has no HTML server — only HTTP API endpoints above. No `/` or dashboard is served from the camera.

---

### Camera Node (Port 81 - Stream Server)

| Endpoint | Method | Purpose | Response |
|----------|--------|---------|----------|
| `/stream` | GET | MJPEG video stream | `multipart/x-mixed-replace` |

**Design Choice:** Separate port for stream prevents blocking of API endpoints

---

## I2C Communication

### Overview

**I2C (Inter-Integrated Circuit)** is used for high-speed communication between Base ESP32 and motor driver.

#### Design Choice: Why I2C?

1. **Multi-Device**: Single bus for multiple motors
2. **Addressable**: Unique 7-bit address (0x34)
3. **Speed**: 40 kHz (matched to motor driver capability)
4. **Simplicity**: Only 2 wires (SDA, SCL)

---

### Register Map

**Motor Driver (Address: 0x34)**

| Register | Purpose | Type | Description |
|----------|---------|------|-------------|
| `0x14` | Motor Type | `uint8_t` | 3 = 4-wheel mecanum |
| `0x15` | Polarity | `uint8_t` | 0 = normal, 1 = inverted |
| `0x33` | Fixed Speed | `int8_t[4]` | Speed for M1-M4 (-100 to 100) |
| `0x3C` | Encoder Total | `int32_t[4]` | Cumulative encoder counts |

---

### I2C Communication Functions

#### Write Bytes

```c
bool writeBytes(uint8_t reg, uint8_t *data, size_t len) {
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(reg);  // Register address
    for (size_t i = 0; i < len; i++)
        Wire.write(data[i]);  // Payload
    return (Wire.endTransmission() == 0);  // ACK received?
}
```

#### Read Encoders

```c
bool readEncoders(int32_t *data) {
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(REG_ENCODER_TOTAL);
    if (Wire.endTransmission() != 0) return false;
    
    if (Wire.requestFrom(I2C_ADDR, 16) != 16) return false;
    
    for (int i = 0; i < 4; i++) {
        uint32_t temp = 0;
        temp |= (uint32_t)Wire.read();        // Byte 0 (LSB)
        temp |= (uint32_t)Wire.read() << 8;  // Byte 1
        temp |= (uint32_t)Wire.read() << 16; // Byte 2
        temp |= (uint32_t)Wire.read() << 24; // Byte 3 (MSB)
        data[i] = (int32_t)temp;  // Interpret as signed
    }
    return true;
}
```

**Design Choice:** Little-endian byte order (ESP32 native format)

---

## Protocol Design Rationale

### 1. Hybrid Communication Architecture

**Why Not Pure ESP-NOW?**
- ESP-NOW lacks built-in encryption (security concern for production)
- No built-in service discovery
- Limited to 250-byte payloads

**Why Not Pure WiFi/TCP?**
- TCP handshake adds latency (unacceptable for real-time control)
- Connection management overhead
- IP routing complexity in ad-hoc networks

**Solution:** Hybrid approach
- **ESP-NOW**: Real-time control (commands, telemetry)
- **WebSocket**: Human interface (dashboard, debugging)
- **HTTP**: Configuration & file serving

---

### 2. Asynchronous Command Queue (Arm Node)

**Problem:** ESP-NOW callbacks run in WiFi task (core 0), but servo movements block on core 1.

**Solution:** Ring buffer with critical section protection

```c
#define CMD_Q_DEPTH 8
static char cmdQueue[CMD_Q_DEPTH][10];
static uint8_t cmdQHead = 0;  // Written by WiFi task (core 0)
static uint8_t cmdQTail = 0;  // Written by loop (core 1)
static portMUX_TYPE cmdMux = portMUX_INITIALIZER_UNLOCKED;

bool enqueueCmd(const char* cmd) {
    portENTER_CRITICAL_ISR(&cmdMux);
    // ... add to queue
    portEXIT_CRITICAL_ISR(&cmdMux);
}
```

**Design Choice:** SPSC (Single Producer Single Consumer) lock-free queue for minimum latency

---

### 3. Temporal Tracking (Camera Node)

**Problem:** QR codes may be temporarily occluded or detection may fail.

**Solution:** Predictive tracking with confidence decay

```c
static bool track_predict_only(uint32_t now_ms) {
    uint32_t age = now_ms - g_track.last_obs_ms;
    float conf = expf(-((float)age) / TRACK_DECAY_TAU_MS);
    g_track.confidence = conf;
    g_track.det.estimated = true;
    
    if (age > TRACK_MAX_HOLD_MS || conf < TRACK_MIN_CONF) {
        g_track.active = false;  // Give up
        return false;
    }
    return true;
}
```

**Design Choice:** Exponential decay models sensor reliability degradation over time

---

### 4. Mecanum Wheel Mixing

**Problem:** Convert (throttle, steering, rotation) to individual wheel speeds.

**Solution:** Linear mixing matrix

```c
void computeMecanumSpeeds(int8_t throttle, int8_t steering, int8_t rotation, int8_t speeds[4]) {
    int16_t fl =  (int16_t)throttle - (int16_t)steering + (int16_t)rotation;
    int16_t fr = -(int16_t)throttle - (int16_t)steering + (int16_t)rotation;
    int16_t bl = -(int16_t)throttle + (int16_t)steering + (int16_t)rotation;
    int16_t br =  (int16_t)throttle + (int16_t)steering + (int16_t)rotation;
    // ... constrain to [-100, 100]
}
```

**Design Choice:** Simple linear algebra (no iterative solvers needed)

---

## Performance Characteristics

### Latency Budget

| Operation | Typical Latency | Worst Case |
|-----------|-----------------|------------|
| ESP-NOW send | 2-3 ms | 5 ms |
| ESP-NOW receive callback | < 1 ms | 2 ms |
| WebSocket message | 10-20 ms | 50 ms |
| I2C write | < 1 ms | 1 ms |
| Encoder read | < 1 ms | 2 ms |
| QR detection (camera) | 80-120 ms | 200 ms |
| Inverse kinematics (arm) | < 1 ms | 2 ms |

---

### Throughput

| Interface | Payload Size | Max Throughput |
|-----------|--------------|----------------|
| ESP-NOW | 250 bytes | ~1 Mbps |
| WebSocket | < 200 bytes | ~100 msgs/sec |
| I2C (40 kHz) | 16 bytes/cmd | ~2.5 Kbps |

---

## Security Considerations

### Current Implementation (Development)

- **No encryption**: ESP-NOW packets sent in cleartext
- **No authentication**: Any device on channel 11 can send packets
- **Static MACs**: Predictable device identities

### Recommendations for Production

1. **Enable ESP-NOW encryption**:
   ```c
   peerInfo.encrypt = true;
   esp_now_set_pmk("premasterkey");  // Set pre-shared key
   ```

2. **Implement MAC whitelisting**:
   ```c
   if (memcmp(mac_addr, knownMAC, 6) != 0) return;  // Reject unknown
   ```

3. **Add packet counters** to prevent replay attacks

4. **Use WebSocket Secure (WSS)** for dashboard

---

## Troubleshooting

### Common Issues

#### 1. ESP-NOW Packets Not Received

**Symptom:** `OnDataRecv` callback never fires

**Causes:**
- Wrong callback signature (ESP-IDF 5.x compatibility)
- WiFi channel mismatch
- Peer not added correctly
- HTTP server task interference

**Debug Steps:**
```bash
# Monitor serial output
screen /dev/ttyUSB0 115200  # Linux
putty COM15 115200            # Windows

# Look for:
# "ESP-NOW INIT FAILED"
# "Peer addition failed"
# "recv_cb: FAILED"
```

---

#### 2. WebSocket Connection Fails

**Symptom:** Phone cannot connect to `ws://192.168.4.1/ws`

**Causes:**
- Base AP not started
- Web server not started (ESP-NOW init failure)
- PROGMEM string not served correctly

**Debug Steps:**
```bash
# Try HTTP health endpoint
curl http://192.168.4.1/healthz

# If no response → web server not running
# Check serial: "Web server started on port 80"
```

---

#### 3. I2C Communication Fails

**Symptom:** `readEncoders()` returns false, motor commands ignored

**Causes:**
- Wrong I2C pins (SDA=21, SCL=22)
- Motor driver not powered
- I2C clock stretch timeout

**Debug Steps:**
```c
Wire.setTimeOut(200);  // Increase timeout
if (!readEncoders(data)) {
    Serial.println("[ERR] I2C read failed");
}
```

---

## References

1. **Espressif ESP-NOW Documentation**: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html
2. **WebSocket Protocol (RFC 6455)**: https://tools.ietf.org/html/rfc6455
3. **I2C Specification**: https://www.nxp.com/docs/en/user-guide/UM10204.pdf
4. **AsyncWebServer Library**: https://github.com/mathieucarbou/ESPAsyncWebServer

---

## Appendices

### A. Complete Packet Definitions

See `base.cpp`, `arm/arm.cpp`, `camera/src/main.cpp` for complete struct definitions.

### B. Serial Debug Messages

All nodes output debug information at 115200 baud. Key messages:

| Message | Meaning |
|---------|---------|
| `[ESP-NOW] INIT FAILED` | ESP-NOW initialization failed |
| `[WS] Client #N connected` | WebSocket client connected |
| `[CAM] Pose: valid=1` | Valid QR pose detected |
| `[ARM] Q+: H` | Command enqueued on Arm |
| `[AUTO] Starting alignment` | Autonomous mode started |

### C. WiFi Channel Allocation

| Node | Mode | Channel | Purpose |
|------|------|---------|---------|
| Base | AP | 11 | WiFi AP + ESP-NOW hub |
| Arm | STA | 11 | Connects to Base AP |
| Camera | STA | 11 | Connects to Base AP |

> **ESP-NOW uses the Base AP MAC** (`80:F3:DA:42:3E:5D`), not the STA MAC (`...3E:5C`). On ESP32, AP MAC = STA MAC + 1. All peer arrays in `arm.cpp` and `camera/src/main.cpp` must use the AP MAC.

---

**End of Document**
