# Code Simplification Plan - Remove RTOS

## Goal
Remove all FreeRTOS dependencies and simplify code using:
- State machines instead of RTOS tasks
- `millis()` based timing instead of `delay()` and `vTaskDelay()`
- Simple variables instead of mutexes/semaphores

## Current RTOS Usage

### 1. firmware/cam_stream/src/main.cpp (HIGH COMPLEXITY)
**RTOS Features Used:**
- `xTaskCreatePinnedToCore()` - QR processing task (line 1609)
- `vTaskDelay()` - Task yielding (line 1616)
- 4 Mutexes: `g_udp_mutex`, `g_platform_mutex`, `g_cam_mutex`, `g_data_mutex`
- `SemaphoreHandle_t` declarations (lines 34, 127, 214, 216)

**Simplification Approach:**
- Remove `qr_task` FreeRTOS task
- Move QR processing to `loop()` with non-blocking timing
- Replace mutexes with simple flags OR ensure single-threaded access
- Use `millis()` for timing instead of `vTaskDelay()`

### 2. base.ino (MEDIUM COMPLEXITY)
**RTOS Features Used:**
- 1 Mutex: `udpMutex` (lines 32, 893)
- Used in `sendUdpPacket()` to protect UDP writes

**Simplification Approach:**
- Remove `udpMutex`
- UDP sends are already atomic in single-threaded context
- If protection needed, use `bool udp_sending` flag

### 3. arm/arm.ino (LOW COMPLEXITY - ALREADY SIMPLE)
**RTOS Features Used:** NONE
**Issue:** Blocking `delay(10)` in servo interpolation loop (line 207)

**Simplification Approach:**
- Replace blocking delays with non-blocking `millis()` based state machine
- Already well-structured for simple conversion

## Implementation Order
1. **cam_stream/main.cpp** - Most complex, highest impact
2. **base.ino** - Medium complexity
3. **arm/arm.ino** - Least complex, just fix blocking delays

## Non-Blocking Pattern Template

Instead of:
```cpp
void process_qr_frame() {
    // blocking QR processing
    delay(100);
}
```

Use:
```cpp
unsigned long last_qr_time = 0;
void loop() {
    if (millis() - last_qr_time > 100) {
        process_qr_frame();
        last_qr_time = millis();
    }
}
```

## State Machine Example

Instead of RTOS task:
```cpp
// RTOS version
xTaskCreate(qr_task, "qr", 32768, NULL, 1, NULL);
void qr_task(void*) {
    while(true) {
        process_qr_frame();
        vTaskDelay(1);
    }
}
```

Use state machine:
```cpp
enum State { IDLE, PROCESSING_QR, SENDING_UDP };
State currentState = IDLE;
unsigned long stateStartTime = 0;

void loop() {
    switch(currentState) {
        case IDLE:
            if (shouldStartQR()) {
                currentState = PROCESSING_QR;
                stateStartTime = millis();
            }
            break;
        case PROCESSING_QR:
            if (processQRStep()) { // Non-blocking step
                currentState = SENDING_UDP;
            }
            break;
        // ...
    }
}
```
