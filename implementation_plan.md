# 🔴 Glitch Communication Protocol — Systematic Debug Audit

Rigorous, layer-by-layer analysis of every inter-node communication pathway in the Glitch robot using embedded-systems fault methodology: **enumerate → classify → trace data path → identify failure mode → rank severity**.

---

## Audit Scope

| Layer | Protocol | Path | Files Analyzed |
|-------|----------|------|----------------|
| L1 | ESP-NOW | Base ↔ Arm | [base.ino](file:///d:/Glitch_Codes/Glitch_CodeBase/base.ino), [arm.ino](file:///d:/Glitch_Codes/Glitch_CodeBase/arm/arm.ino), [arm.cpp](file:///d:/Glitch_Codes/Glitch_CodeBase/arm/arm.cpp) |
| L2 | ESP-NOW | Base ↔ Camera | [base.ino](file:///d:/Glitch_Codes/Glitch_CodeBase/base.ino), [main.cpp](file:///d:/Glitch_Codes/Glitch_CodeBase/camera/src/main.cpp) |
| L3 | HTTP POST/GET | Dashboard → Base | [base.ino](file:///d:/Glitch_Codes/Glitch_CodeBase/base.ino) (embedded HTML+JS) |
| L4 | I2C | Base → Motor Driver | [base.ino](file:///d:/Glitch_Codes/Glitch_CodeBase/base.ino) |
| L5 | I2C | Arm → PCA9685 | [arm.cpp](file:///d:/Glitch_Codes/Glitch_CodeBase/arm/arm.cpp) |
| L6 | Build System | PlatformIO | [platformio.ini](file:///d:/Glitch_Codes/Glitch_CodeBase/platformio.ini) |

---

## 🚨 CRITICAL Findings (Will Cause Silent Failures)

### C1. MAC Address Mismatch — `arm.ino` uses WRONG Base MAC (STA instead of AP)

> [!CAUTION]
> **Severity: CRITICAL** — Arm → Base ESP-NOW packets silently dropped. Arm status will NEVER reach base.

| File | Line | Value | Interface |
|------|------|-------|-----------|
| [arm.ino:81](file:///d:/Glitch_Codes/Glitch_CodeBase/arm/arm.ino#L81) | `baseAddress` | `0x80,0xF3,0xDA,0x42,0x3E,**0x5C**` | **STA** ❌ |
| [arm.cpp:81](file:///d:/Glitch_Codes/Glitch_CodeBase/arm/arm.cpp#L81) | `baseAddress` | `0x80,0xF3,0xDA,0x42,0x3E,**0x5D**` | **AP** ✅ |
| [main.cpp:24](file:///d:/Glitch_Codes/Glitch_CodeBase/camera/src/main.cpp#L24) | `baseAddress` | `0x80,0xF3,0xDA,0x42,0x3E,**0x5D**` | **AP** ✅ |

**Root cause:** The Base operates ESP-NOW on `WIFI_IF_AP`. AP MAC = STA MAC + 1. `arm.ino` still uses the old STA MAC `0x5C`. The corrected `arm.cpp` has `0x5D`.

**Impact:**
- `sendArmStatus()` in `arm.ino` sends to wrong MAC → Base's `OnDataRecv` checks `memcmp(mac, armAddress, 6)` which is the arm's MAC (not base's), so the *receive* still works, BUT the arm's `esp_now_send(baseAddress, ...)` targets the wrong peer
- All arm → base status packets (`ArmStatus busy/idle`) are **silently dropped**
- The dashboard always shows stale `arm_busy` state

**But wait — which file gets compiled?** See C2 below.

---

### C2. Build System Compiles `arm.cpp`, Not `arm.ino` — Divergent Source Files

> [!CAUTION]
> **Severity: CRITICAL** — You have TWO versions of arm firmware that are diverging. Only one gets compiled.

```ini
# platformio.ini line 16
[env:arm]
build_src_filter = -<*> +<arm/arm.cpp>
```

PlatformIO compiles **only** [arm.cpp](file:///d:/Glitch_Codes/Glitch_CodeBase/arm/arm.cpp). The file [arm.ino](file:///d:/Glitch_Codes/Glitch_CodeBase/arm/arm.ino) is **dead code** — never compiled, never flashed.

**Key divergences between the two files:**

| Aspect | arm.ino (DEAD) | arm.cpp (ACTIVE) |
|--------|----------------|------------------|
| Base MAC | `0x5C` ❌ (STA) | `0x5D` ✅ (AP) |
| `OnDataRecv` signature | Old: `(const uint8_t *mac_addr, ...)` ❌ | New: `(const esp_now_recv_info_t *info, ...)` ✅ |
| ESP-NOW send stats | Not tracked | `espnowTxOk`, `espnowTxFail` ✅ |
| Heartbeat status send | Missing ❌ | Sends idle status every 5s ✅ |
| WiFi reconnect vars | Local static inside function | Global static (cleaner) |
| `goHome()` in setup | Calls `goHome()` directly (blocks WiFi) | Deferred to first `loop()` iteration ✅ |
| `sendArmStatus` | No error check | Checks `esp_err_t` and logs ✅ |

**Impact:** The `arm.ino` is a landmine. If someone edits `arm.ino` thinking it's the active source, their changes silently vanish. If someone opens the Arduino IDE (which uses `.ino`), it compiles the WRONG file.

**Fix:** Delete `arm.ino` or rename it to `arm.ino.DEAD.bak` to prevent confusion.

---

### C3. `arm.ino` `OnDataRecv` Signature is Wrong for ESP-IDF 5.x

> [!WARNING]
> **Severity: HIGH** (mitigated because `arm.cpp` is compiled instead, but if anyone flashes `arm.ino`…)

```cpp
// arm.ino:480 — WRONG for Arduino-ESP32 3.x / ESP-IDF 5.x
void OnDataRecv(const uint8_t *mac_addr,
                const uint8_t *incomingData,
                int len) {
```

```cpp
// arm.cpp:487 — CORRECT
void OnDataRecv(const esp_now_recv_info_t *info,
                const uint8_t *incomingData,
                int len) {
```

The old signature will cause a **compilation failure** on modern Arduino-ESP32 3.x. This is a type mismatch on `esp_now_register_recv_cb()`.

---

## ⚠️ HIGH Findings (Robustness & Reliability Issues)

### H1. `strcpy()` Buffer Overflow in `sendCommandToArm()`

> [!WARNING]
> **Severity: HIGH** — Classic embedded buffer overflow. Can corrupt adjacent memory.

```cpp
// base.ino:199
void sendCommandToArm(const char *cmd) {
  strcpy(armMessage.command, cmd);  // ← NO bounds check!
```

`armMessage.command` is a `char[10]` buffer. If `cmd` exceeds 9 characters (including null terminator), `strcpy` writes past the buffer. The `handleCommand()` function passes user-derived strings like `arg.c_str()` at [base.ino:568](file:///d:/Glitch_Codes/Glitch_CodeBase/base.ino#L568):

```cpp
} else {
    sendCommandToArm(arg.c_str());  // arg could be ANY length from HTTP POST
}
```

A malicious or buggy dashboard POST with a long `arg` value corrupts the stack.

**Fix:** Replace `strcpy` with `strncpy`:
```cpp
strncpy(armMessage.command, cmd, sizeof(armMessage.command) - 1);
armMessage.command[sizeof(armMessage.command) - 1] = '\0';
```

---

### H2. `volatile` Struct Read is Not Atomic — Torn Reads Possible

> [!WARNING]
> **Severity: HIGH** — Non-atomic read of multi-byte volatile struct from ISR context.

In [base.ino:90-91](file:///d:/Glitch_Codes/Glitch_CodeBase/base.ino#L90-L91):
```cpp
static volatile bool cameraPoseReceived = false;
static volatile PoseReply lastPoseReply;  // 24 bytes!
```

The ISR callback (`OnDataRecv`) writes all 24 bytes via `memcpy`. The `loop()` reads them at [base.ino:1027-1028](file:///d:/Glitch_Codes/Glitch_CodeBase/base.ino#L1027-L1028):
```cpp
PoseReply reply;
memcpy(&reply, (const void *)&lastPoseReply, sizeof(reply));
```

**Problem:** On ESP32 dual-core, the WiFi callback runs on core 0 while `loop()` runs on core 1. A `memcpy` of 24 bytes is NOT atomic. If the WiFi task interrupts mid-copy, `reply` contains a mix of old and new data — **torn read**.

The arm firmware ([arm.cpp](file:///d:/Glitch_Codes/Glitch_CodeBase/arm/arm.cpp)) has the SAME problem with `incomingCameraPose` (line 78) — no spinlock or critical section protecting the 24-byte struct copy.

**Fix:** Use `portENTER_CRITICAL` / `portEXIT_CRITICAL` around both the write (in callback) and the read (in loop), or use a FreeRTOS queue.

---

### H3. Autonomous Mode Blocks `loop()` for 12+ Seconds — Starves WebServer

> [!WARNING]
> **Severity: HIGH** — During autonomous pickup, the web dashboard goes unresponsive.

In [base.ino:1090-1145](file:///d:/Glitch_Codes/Glitch_CodeBase/base.ino#L1090-L1145), the autonomous mode runs a blocking loop:

```cpp
// base.ino:1123-1128 — blocking wait after sending arm command
unsigned long t0 = millis();
while (millis() - t0 < 12000) {
  server.handleClient();  // ← patched, but still blocks for 12s PER TARGET
  if (!autonomousMode) break;
  delay(50);
}
```

For 3 color targets, this blocks for up to **36 seconds** in a single `loop()` iteration. During `alignToQR()` calls, there are additional `waitForCameraPose()` calls with 3-5 second timeouts. Total worst-case block: **~50 seconds**.

While `server.handleClient()` is called inside the wait loops, the `alignToQR()` function has `delay(120)` and `delay(200)` pauses ([base.ino:429,443](file:///d:/Glitch_Codes/Glitch_CodeBase/base.ino#L429-L443)) where the web server is NOT serviced. HTTP requests during these windows will timeout.

---

### H4. Camera Node Static IP Conflicts with NODES.md Documentation

> [!IMPORTANT]
> **Severity: MEDIUM-HIGH** — Documentation says `.202`, code says `.100`

| Source | Camera IP |
|--------|-----------|
| [NODES.md](file:///d:/Glitch_Codes/Glitch_CodeBase/NODES.md) | `192.168.4.202` |
| [main.cpp:1388](file:///d:/Glitch_Codes/Glitch_CodeBase/camera/src/main.cpp#L1388) | `192.168.4.100` |

```cpp
// camera/src/main.cpp:1388
IPAddress staticIP(192, 168, 4, 100);  // ← doesn't match NODES.md
```

If any other component (scripts, monitoring tools, future UDP code) targets `.202`, it will fail. This also means the `NODES.md` UDP port 4210 documentation is misleading — the camera doesn't listen on UDP port 4210 at all, it uses ESP-NOW + HTTP.

---

### H5. `ScanRequest.mode=1` (Platform Scan) is Accepted but Never Processed

> [!IMPORTANT]
> **Severity: MEDIUM** — Dashboard sends platform scan requests that do nothing.

In [base.ino:594-597](file:///d:/Glitch_Codes/Glitch_CodeBase/base.ino#L594-L597):
```cpp
} else if (arg == "PLAT") {
    sendScanRequest(1);  // mode=1 = platform scan
```

But in [main.cpp:1035-1071](file:///d:/Glitch_Codes/Glitch_CodeBase/camera/src/main.cpp#L1035-L1071), the scan response handler ALWAYS runs QR detection regardless of `g_scan_mode`. The `mode` field is stored but never read during processing:

```cpp
// Camera stores mode but never uses it:
g_scan_mode = req.mode;  // stored...
// process_qr_frame() always runs QR pipeline, never calls detect_platform()
```

The `platform_detect.h` API exists, but `detect_platform()` is never called in `main.cpp`. Platform detection is **dead code**.

---

## ⚡ MEDIUM Findings (Correctness & Maintainability)

### M1. `SSTEP` Command Parsing is Fragile and Undocumented

In [base.ino:571-575](file:///d:/Glitch_Codes/Glitch_CodeBase/base.ino#L571-L575):
```cpp
} else if (cmd == "SSTEP") {
    int joint = arg.charAt(1) - '1';  // assumes format like "J1,5"
    int val = arg.substring(3).toInt();
    char armCmd[10];
    snprintf(armCmd, sizeof(armCmd), "SV:%d:%d", joint, val);
```

But the dashboard JS sends at [base.ino:878](file:///d:/Glitch_Codes/Glitch_CodeBase/base.ino#L878):
```js
send({cmd:'SSTEP', arg:stepJoints[j]+','+(dir*deg)});
// e.g. arg = "J1,5" or "J3,-10"
```

**Issues:**
- `arg.charAt(1) - '1'` assumes joint name is always 2 chars. `J1` → `charAt(1)='1'` → joint 0. Works for J1-J5 but has no validation.
- `arg.substring(3)` skips the comma. `J1,5` → `substring(3)` = `5`. `J1,-10` → `substring(3)` = `-10`. But `J10` would break (joint index > 9 doesn't exist but would parse wrong).
- The resulting `SV:idx:dir` command is forwarded to the arm, but the arm's `stepServo` interprets `dir` as **absolute degrees to add**, not as a step direction multiplied by step size. This works because the dashboard pre-computes `dir*deg`.

### M2. `arm_pose_color_from_text()` Has False-Positive Risk

In [arm_pose_link.cpp:74-98](file:///d:/Glitch_Codes/Glitch_CodeBase/camera/src/arm_pose_link.cpp#L74-L98):
```cpp
// First checks start-of-string:
if (starts_with_ci(text, text_len, "R") || ...) return ARM_COLOR_R;
```

A QR code starting with `"R"` (like `"ROBOT"` or `"RESERVED"`) gets classified as RED. The fallback scan at line 84-98 is even worse — any text containing `R`, `G`, or `B` anywhere gets matched (first one found wins). A QR reading `"TABLE"` would match `B` → BLUE.

### M3. `conflicts_tobe_resolved.md` Documents a WebSocket Binary Protocol That No Longer Exists

The [conflicts_tobe_resolved.md](file:///d:/Glitch_Codes/Glitch_CodeBase/conflicts_tobe_resolved.md) extensively documents a `ControlState` binary WebSocket protocol with Fletcher-16 checksums, 25Hz updates, and deadman switches. **None of this exists in the current codebase.** The current `base.ino` uses:
- HTTP POST `/cmd` with JSON body (not WebSocket binary)
- HTTP GET `/status` polling at 500ms (not WebSocket push)
- No deadman switch, no checksum, no seq counter

This stale documentation is actively misleading for anyone onboarding.

### M4. JSON Status Endpoint Builds Strings by Concatenation — Fragile & Injection-Risk

In [base.ino:980-998](file:///d:/Glitch_Codes/Glitch_CodeBase/base.ino#L980-L998), JSON is built via `String` concatenation:
```cpp
String json = "{";
json += "\"arm_busy\":" + String(...);
```

This is both slow (repeated heap allocations on ESP32 with limited RAM) and fragile — if any float value is `NaN` or `Inf`, the JSON becomes invalid, breaking the dashboard.

### M5. Camera `data_handler()` Allocates 4KB on Heap Per Request

In [main.cpp:1148](file:///d:/Glitch_Codes/Glitch_CodeBase/camera/src/main.cpp#L1148):
```cpp
char *buf = (char *)malloc(4096);
```

Under rapid polling or concurrent requests, this can fragment the ESP32-S3 heap. A static buffer with mutex would be safer.

### M6. No Heartbeat/Keepalive from Camera → Base

The arm sends periodic idle status ([arm.cpp:637-639](file:///d:/Glitch_Codes/Glitch_CodeBase/arm/arm.cpp#L637-L639)):
```cpp
if (WiFi.status() == WL_CONNECTED) {
    sendArmStatus(false);  // heartbeat every 5s
}
```

The camera has **no equivalent**. The base has no way to know if the camera is alive, crashed, or disconnected. `g_scan_miss_counter` only increments when scans are actively requested.

### M7. `waitForCameraPose()` Clears Flag Before Send — Race Condition

In [base.ino:385-387](file:///d:/Glitch_Codes/Glitch_CodeBase/base.ino#L385-L387):
```cpp
static bool waitForCameraPose(unsigned long timeout_ms) {
    cameraPoseReceived = false;  // clear
    sendScanRequest(0);          // request
```

If a stale/delayed PoseReply from a previous request arrives between the flag clear and the new request send, it gets correctly ignored. BUT: if it arrives between `sendScanRequest()` and the `while` loop, the flag gets set to `true` before the new response arrives, and the function returns the **old** data.

The `task_id` field exists to solve this, but it's never validated — `waitForCameraPose` doesn't check if `lastPoseReply.task_id` matches the request's task ID.

---

## 💡 LOW Findings (Quality of Life / Tech Debt)

### L1. No ESP-NOW Send Callback on Base for Camera Peer
The base's `OnDataSent` prints SUCCESS/FAILED but doesn't track per-peer delivery. No counter distinguishes arm vs camera send failures.

### L2. `delay(500)` in WiFi Setup
[base.ino:908](file:///d:/Glitch_Codes/Glitch_CodeBase/base.ino#L908) has a `delay(500)` after `Wire.begin`. Undocumented — may not be needed. Every ms matters in boot time for competition.

### L3. Camera WiFi Retry Interval is 10s vs Arm's 5s
[main.cpp:1504](file:///d:/Glitch_Codes/Glitch_CodeBase/camera/src/main.cpp#L1504): Camera retries every 10s. [arm.cpp:547](file:///d:/Glitch_Codes/Glitch_CodeBase/arm/arm.cpp#L547): Arm retries every 5s. Inconsistent but not broken.

### L4. `servoAngle[]` on Base Drifts from Arm's Actual Angles
[base.ino:159](file:///d:/Glitch_Codes/Glitch_CodeBase/base.ino#L159): Base maintains a shadow `servoAngle[4]` but never receives confirmation from the arm. After any arm preset sequence, the base's shadow is stale.

### L5. `arm_pose_link.cpp` UDP Infrastructure is Unused
The camera firmware includes [arm_pose_link.cpp](file:///d:/Glitch_Codes/Glitch_CodeBase/camera/src/arm_pose_link.cpp) with full UDP client code (`WiFiUDP`, `arm_pose_link_begin`, `arm_pose_link_send`). This is **never called** — ESP-NOW is used instead. Dead code adding ~3KB to flash.

### L6. Dashboard `/status` Polled at 500ms But Camera QR FPS is ~4fps
The dashboard polls at 2Hz but the QR pipeline runs at ~4fps. Half the polls return stale data.

---

## User Review Required

> [!IMPORTANT]
> **Two questions need your input before I fix anything:**

### Q1: Which arm firmware is canonical?
PlatformIO compiles `arm.cpp`. Should I **delete `arm.ino`** to eliminate the divergence, or do you use `arm.ino` separately in Arduino IDE?

### Q2: Camera static IP — which is correct?
- [NODES.md](file:///d:/Glitch_Codes/Glitch_CodeBase/NODES.md) says `192.168.4.202`
- [main.cpp](file:///d:/Glitch_Codes/Glitch_CodeBase/camera/src/main.cpp#L1388) hardcodes `192.168.4.100`
- Which should it be?

---

## Proposed Fix Priority

| # | Finding | Severity | Effort | Fix |
|---|---------|----------|--------|-----|
| 1 | C1 | 🔴 CRITICAL | 1 line | Fix `arm.ino` base MAC to `0x5D` (or delete the file) |
| 2 | C2 | 🔴 CRITICAL | File mgmt | Delete or archive `arm.ino` to prevent confusion |
| 3 | H1 | 🟠 HIGH | 2 lines | Replace `strcpy` with `strncpy` in `sendCommandToArm` |
| 4 | H2 | 🟠 HIGH | ~15 lines | Add `portMUX` spinlock around volatile struct read/write in base + arm |
| 5 | H7 | 🟠 HIGH | ~5 lines | Validate `task_id` in `waitForCameraPose()` to prevent stale data |
| 6 | H4 | 🟡 MEDIUM | 1 line | Align camera IP to documentation (or update docs) |
| 7 | H5 | 🟡 MEDIUM | ~30 lines | Wire `platform_detect()` into the scan pipeline when `mode=1` |
| 8 | M2 | 🟡 MEDIUM | ~10 lines | Tighten `arm_pose_color_from_text()` matching logic |
| 9 | M3 | 🟡 MEDIUM | Doc update | Update `conflicts_tobe_resolved.md` to match current HTTP protocol |
| 10 | M6 | 🟡 MEDIUM | ~10 lines | Add camera heartbeat ESP-NOW packet |
| 11 | H3 | 🟡 MEDIUM | Architecture | Refactor autonomous mode to non-blocking state machine |

## Verification Plan

### Automated Tests
- Compile all 3 environments: `pio run -e base && pio run -e arm`
- Camera builds separately via its own `platformio.ini`
- `sizeof()` static assertions for all packed structs to catch padding changes

### Manual Verification
- Flash arm with corrected `arm.cpp`, monitor serial — confirm `[ARM] ESP-NOW send` status packets reach base
- Send long string via dashboard custom ARM command — confirm no crash (strcpy fix)
- Trigger autonomous mode — confirm dashboard remains responsive
- Run platform scan from dashboard — confirm camera processes mode=1 differently
