# Conflicts To Be Resolved

Reference document for Project Glitch. Covers communication protocols, known conflicts, and intentional simplifications.

---

## Communication Pipeline
Phone (controller.html)

 ↓ WebSocket (TCP, 10-byte binary, 25Hz)

Base ESP32

↓ I2C (400kHz, 4 bytes) — motors

↓ ESP-NOW (WiFi, 10-byte ASCII, on-demand) — arm commands

Arm ESP32

↓ I2C (400kHz, PWM) — PCA9685 servos
---

## Protocol 1: WebSocket (Phone → Base)

**Packet: `ControlState` — 10 bytes, packed, little-endian**

| Offset | Size | Field | Notes |
|--------|------|-------|-------|
| 0 | 4 | seq | uint32, monotonically increasing |
| 4 | 1 | throttle | int8, -100 to 100 |
| 5 | 1 | steering | int8, -100 to 100 |
| 6 | 1 | rotation | int8, -100 to 100 |
| 7 | 1 | arm_action | uint8, 0=idle, 1-8=presets, 10-19=servo step |
| 8 | 2 | checksum | uint16 LE, Fletcher-16 over bytes 0-7 |

**JS packing** (`controller.html`):
```js
v.setUint32(0, state.seq, true);
v.setInt8(4, state.throttle);
v.setInt8(5, state.steering);
v.setInt8(6, state.rotation);
v.setUint8(7, state.arm_action);
v.setUint16(8, (sum2 << 8) | sum1, true);
```

**C++ unpacking** (`base.ino`):
```cpp
struct __attribute__((packed)) ControlState {
  uint32_t seq;
  int8_t throttle;
  int8_t steering;
  int8_t rotation;
  uint8_t arm_action;
  uint16_t checksum;
};
```

**arm_action encoding:**
- 0 = idle (no action)
- 1 = Home, 2 = GTF, 3 = BTF, 4 = RTF, 5 = GTC, 6 = BTC, 7 = RTC, 8 = Scan
- 10-19 = servo step: `10 + (idx * 2) + (dir === 1 ? 1 : 0)`
  - idx 0-4 (base, shoulder, elbow, wrist, gripper)
  - dir -1 or 1
  - Examples: 10=SV:0:-1, 11=SV:0:1, 19=SV:4:1

**Base handling:**
- Validates Fletcher-16 checksum
- Only accepts packets with `seq > g_lastRxSeq` (prevents reordering)
- Copies to `g_controlState` under `portENTER_CRITICAL`
- Sets `g_deadmanActive = true`, records `g_lastPacketTime`

**Known conflict — arm_action lifecycle:** `INTENTIONAL`
Phone sets `arm_action` then clears it after 100ms via `setTimeout`. Base reads and clears it in `controlLoop()` under lock. If base doesn't read it within 100ms, the action is lost. This is intentional — prevents stuck commands.

**Known conflict — Fletcher-16 endianness:** `NEEDS VERIFICATION`
JS packs as `setUint16(8, (sum2 << 8) | sum1, true)`. On ESP32 (little-endian), the struct reads byte 8 as low and byte 9 as high, producing `(sum2 << 8) | sum1` — this should match. Must be confirmed once with a known packet before field use. Send a fixed packet (seq=1, all fields=0), log raw received checksum bytes and computed value on base, confirm equal. If they differ, swap `sum1`/`sum2` in the JS call.

---

## Protocol 2: ESP-NOW (Base → Arm)

**Packet: `ArmCommand` — 10 bytes, plain ASCII**

```cpp
typedef struct {
  char command[10];
} ArmCommand;
```

**Commands:** `H`, `S`, `GTF`, `BTF`, `RTF`, `GTC`, `BTC`, `RTC`, `SV:idx:dir`

**Base sends:**
```cpp
strncpy(armMessage.command, cmd, 9);
armMessage.command[9] = '\0';
esp_now_send(armMacAddress, (uint8_t*)&armMessage, sizeof(armMessage));
```

**Arm receives (in WiFi task callback):**
```cpp
void OnDataRecv(...) {
  ArmCommand msg;
  memcpy(&msg, incomingData, sizeof(msg));
  msg.command[9] = '\0';
  enqueueCmd(msg.command);  // returns immediately
}
```

**Known conflict — no ACK/delivery confirmation:** `INTENTIONAL`
`esp_now_send()` only confirms queueing, not delivery. If the arm is busy (queue full, WiFi congestion), the command is silently dropped. The `onEspNowSend` callback logs failures but has no retry. Acceptable — the phone re-sends at 25Hz anyway.

**Known conflict — arm_action retransmit storm:** `RESOLVED`
The base is level-triggered on `arm_action`. During the 100ms window the phone holds `arm_action` non-zero, `controlLoop()` runs at ~20Hz and ticks 2-3 times. Each tick sees `arm_action != 0` and sends a new ESP-NOW command. The arm queues each one and executes them sequentially — one button press causes 2-3 preset executions.

Fix — make the base edge-triggered. Add to `base.ino`:

```cpp
// Global:
static uint8_t g_lastSentArmAction = 0;

// In controlLoop(), replace the arm_action send block:
uint8_t action = g_controlState.arm_action;
if (action != 0 && action != g_lastSentArmAction) {
    sendArmCommand(decodeArmAction(action));
    g_lastSentArmAction = action;
}
if (action == 0) {
    g_lastSentArmAction = 0;
}
```

Phone-side button disable during execution is still good UX but must not be the sole protection — network timing makes it unreliable on its own.

---

## Protocol 3: I2C (Base → Motor Driver)

**MD02 motor driver at address 0x34, 400kHz**

| Register | Purpose | Data |
|----------|---------|------|
| 0x14 | Motor type | 3 (4-wheel mecanum) |
| 0x15 | Polarity | 0 (normal) |
| 0x33 | Fixed speed | 4 bytes: FL, FR, BL, BR (-100 to 100) |

**Mecanum mixing:**
FL =  throttle - steering + rotation

FR = -throttle - steering + rotation

BL = -throttle + steering + rotation

BR =  throttle + steering + rotation
Constrained to [-100, 100].

**Deadman switch:**
- Phone sends packets at 25Hz (40ms)
- Base checks `millis() - g_lastPacketTime > 200`
- If timeout: `forceStop()` (writes 0,0,0,0 to motor driver)
- 200ms = 5 packet margin at 25Hz

**Known conflict — I2C in WS callback:** `RESOLVED`
`forceStop()` was moved out of the WS disconnect callback. It now runs in `controlLoop()` via `g_pendingStop` flag. The WS callback only sets the flag under `portENTER_CRITICAL`. This prevents blocking I2C inside the network task.

**Hardware — mecanum motor polarity:** `NEEDS VERIFICATION`
Mixing matrix is correct for X-configuration mecanum where positive = forward on all wheels. Register 0x15 = 0 means no firmware compensation — correctness depends entirely on physical wiring. Verify with wheels off the ground:
- Throttle only (50, 0, 0): FL+BR and FR+BL spin in opposite pairs
- Steering only (0, 50, 0): FL+BR spin one direction, FR+BL the other
- Rotation only (0, 0, 50): FL+BL spin one direction, FR+BR the other

If a wheel is inverted, fix via register 0x15 per-wheel polarity bits — do not touch the mixing math.

---

## Protocol 4: I2C (Arm → Servos)

**PCA9685 servo driver, 50Hz PWM**

| Servo | Index | Angle Range | Notes |
|-------|-------|-------------|-------|
| Base | 0 | 0-180 | Rotates whole arm |
| Shoulder | 1 | 10-170 | |
| Elbow | 2 | 10-170 | |
| Wrist | 3 | 50-180 | |
| Gripper | 4 | 30-160 | 30=open, 160=closed |

**`executeSyncMove()` — blocking with yield:**
- Calculates duration from max angular delta × 10ms/degree
- Minimum 300ms per move
- Smooth step via cosine interpolation
- `delay(10)` in loop yields to FreeRTOS (WiFi task can fill command queue)

**Known conflict — blocking during presets:** `INTENTIONAL`
Preset functions call `moveRobot()` multiple times sequentially. Each call blocks in `executeSyncMove()`. `loop()` cannot drain the command queue during this time. However `delay(10)` yields to FreeRTOS so the WiFi task keeps receiving and enqueueing. Commands drain when the preset finishes.

---

## Ring Buffer (Arm Command Queue)

**SPSC (Single Producer, Single Consumer):**
- Producer: `OnDataRecv` callback (WiFi task, core 0) — writes `cmdQHead`
- Consumer: `loop()` (Arduino main, core 1) — writes `cmdQTail`
- No locks needed. `volatile` ensures compiler doesn't cache reads.

**Depth: 8 slots.** Consider bumping to 16 (`#define CMD_QUEUE_SIZE 16`) — costs 80 bytes of RAM, eliminates drop risk during long presets.

**Known conflict — queue full:** `ACCEPTABLE`
If the arm is executing a long preset (5-10s) and commands arrive faster than it can drain them, the queue fills and new commands are dropped with a log message. The phone's 100ms `setTimeout` limits burst to ~2-3 commands per action so this is unlikely in normal use. Bumping to 16 slots reduces risk further.

---

## Missing: Base → Phone Feedback `SHOULD ADD`

All communication is currently unidirectional. The phone has no visibility into arm state, motor driver health, or arm WiFi status. Suggested: a 4-byte status packet sent from base to phone over WebSocket at ~5Hz.

```cpp
struct StatusPacket {
    uint8_t type;         // 0xFF = status marker
    uint8_t arm_busy;     // 1 = arm executing a command
    uint8_t motor_ok;     // 1 = I2C to motor driver healthy
    uint8_t arm_wifi_ok;  // 1 = last ESP-NOW send succeeded
};
```

`motor_ok` and `arm_wifi_ok` require no arm-side changes — both are already available on the base from the I2C result and `onEspNowSend` callback respectively. `arm_busy` requires the arm to send its state back via ESP-NOW — implement that last.

Phone-side uses: disable preset buttons when `arm_busy = 1`, show warning indicators for motor/WiFi faults.

---

## Intentional Simplifications

These are NOT bugs. They are deliberate trade-offs for simplicity.

1. **No ESP-NOW re-init on WiFi reconnect** — ESP-NOW peer channel may go stale if arm WiFi drops and reconnects. In practice it usually recovers fine on the same channel. Adding re-init adds complexity for a rare edge case.
2. **No async servo moves** — `executeSyncMove()` blocks. The arm can't drive motors while moving servos. The command queue absorbs bursts during a move.
3. **No ACK from arm to base** — Fire-and-forget. If delivery fails, the phone re-sends at 25Hz. ACKs would require bidirectional ESP-NOW and a state machine.
4. **arm_action cleared after 100ms** — If base doesn't read it in time, the action is lost. Prevents stuck commands when phone disconnects.
5. **Speed written every 50ms unconditionally** — I2C write of 4 bytes takes ~100μs. At 20Hz that's 0.2% CPU. Not worth the complexity of tracking last-sent values.

---

## Edge Cases to Watch

| Scenario | What happens | Mitigation |
|----------|-------------|------------|
| Phone disconnects mid-preset | Arm completes current preset, then idle | Deadman stops motors on base |
| Preset button pressed twice | Two executions queued back-to-back | Fix retransmit storm — phone button disable is secondary |
| Servo step during preset | Enqueued, executes after preset finishes | User shouldn't mix manual and preset |
| Queue full during long preset | Commands dropped | Bump queue to 16 slots |
| WiFi drops on arm | Commands lost until reconnect | Reduce `checkWifi()` from 5s to 1.5s |
| PCA9685 not found | Arm halts (logs error) | Check I2C wiring — surface via status packet |
| Motor driver not responding | I2C errors logged, robot doesn't move | Check I2C wiring — surface via `motor_ok` status bit |

---

## Open Items

| ID | Status | Item |
|----|--------|------|
| C1 | ✅ RESOLVED | Retransmit storm — base now edge-triggered via g_lastSentArmAction |
| C2 | 🔍 NEEDS VERIFY | Fletcher-16 endianness — test with known packet, document result |
| C3 | 🔍 NEEDS VERIFY | Mecanum motor polarity — bench test all three movement axes |
| C4 | 💡 SHOULD ADD | Base → phone status feedback (motor_ok, arm_wifi_ok, arm_busy) |
| C5 | 💡 CONSIDER | Bump arm command queue from 8 to 16 slots |
| C6 | 💡 CONSIDER | Reduce arm WiFi reconnect interval from 5s to 1.5s |
| C7 | ✅ RESOLVED | forceStop() moved out of WS callback — now via g_pendingStop flag |
| C8 | ✅ INTENTIONAL | arm_action 100ms timeout — prevents stuck commands |
| C9 | ✅ INTENTIONAL | No ACK from arm to base — fire-and-forget acceptable at 25Hz |
| C10 | ✅ INTENTIONAL | Blocking executeSyncMove() — queue absorbs bursts, yields via delay(10) |
| C11 | ✅ RESOLVED | OnDataRecv signature — all three firmware files updated from old `const uint8_t*` to `const esp_now_recv_info_t*` (ESP-IDF 5.x fix) |
| C12 | ✅ RESOLVED | Base MAC: STA (`0x5C`) and AP (`0x5D`) are different interfaces — ESP-NOW uses AP MAC. Firmware is correct. |