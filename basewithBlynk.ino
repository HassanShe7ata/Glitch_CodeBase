// =========================== BASE ESP32 CODE ===========================

// ================= LOCAL BLYNK SERVER =================
// Replace 192.168.X.X with your computer's local IP address
#define BLYNK_PRINT Serial
#define BLYNK_LOCAL_PORT 8080

bool autoTrigger = 0;

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// ================= ESP-NOW SHARED ENUMS =================
// These must match the definitions in camera firmware (main.cpp)
// and arm firmware (unused there but kept for consistency).
enum ArmColorCode : uint8_t {
    ARM_COLOR_UNKNOWN = 0,
    ARM_COLOR_R       = 1,
    ARM_COLOR_G       = 2,
    ARM_COLOR_B       = 3,
};

// Your authentication token (get this from Blynk Legacy server after creating project)
char auth[] = "Kspg0_T5ov2BDlZ3-HMLCJoOoWtlRrqV";

// Local server IP - CHANGE THIS to your computer's IP address
// Find it with: ipconfig (Windows) - look for IPv4 Address
char blynkServer[] = "192.168.5.1";  // <-- CHANGE THIS accordingly
#include <Wire.h>
#include <esp_now.h>

// ================= WIFI =================
char ssid[] = "hassan's-laptop-hotspot";
char pass[] = "12345678";

// ================= ESP NOW =================

// Arm ESP32 MAC Address
uint8_t armAddress[] = {0x68, 0xFE, 0x71, 0x12, 0x5D, 0xA8};


typedef struct struct_message {
    char command[10];
} struct_message;

struct_message armMessage;

esp_now_peer_info_t peerInfo;

// =================== CAMERA ESP-NOW ===================
// ⚠️  MUST UPDATE: Run camera once and read Serial Monitor for "CAMERA MAC: xx:xx:xx:xx:xx:xx"
// Replace the FF placeholder below with the actual 6-byte MAC address.
// ESP-NOW cannot use broadcast (FF:FF:FF:FF:FF:FF) as a peer address.
static uint8_t cameraAddress[] = {0x94, 0xA9, 0x90, 0x08, 0xB2, 0xB8};

// ESP-NOW packet: Base → Camera (scan request)
struct __attribute__((packed)) ScanRequest {
    uint8_t task_id;
    uint8_t mode;        // 0=scan_qr, 1=scan_platform
    uint8_t reserved[2];
};

// ESP-NOW packet: Camera → Base (pose reply)
struct __attribute__((packed)) PoseReply {
    uint8_t task_id;
    uint8_t pose_valid;
    uint8_t color;       // 0=unknown, 1=R, 2=G, 3=B
    uint8_t estimated;
    float tx_mm;
    float ty_mm;
    float tz_mm;
    float yaw_deg;
    float confidence;
};

// Latest camera pose data (updated by ESP-NOW callback)
// Marked volatile because OnDataRecv (ISR callback) writes it
// while alignToQR() / waitForCameraPose() read it in the main loop.
static volatile bool cameraPoseReceived = false;
static volatile PoseReply lastPoseReply;

// Movement watchdog
static bool isMoving = false;
static unsigned long lastMoveTime = 0;
const unsigned long MOVE_TIMEOUT_MS = 3000; // auto-stop after 3 seconds

// --- I2C Registers ---
#define I2C_ADDR           0x34
#define REG_MOTOR_TYPE     0x14
#define REG_MOTOR_PHASE    0x15
#define REG_FIXED_SPEED    0x33
#define REG_ENCODER_TOTAL  0x3C

// --- Pins ---
#define SDA_PIN 21
#define SCL_PIN 22

bool autonomousMode = false;

// --- CALIBRATED TICK CONSTANTS (WEIGHT-AWARE) ---
const float TICKS_FWD_BWD = 5540.0;
const float TICKS_STRAFE  = 6253.0;
const float TICKS_DIAG    = 7875.0;
const float TICKS_ROTATE  = 7200.0;

 int8_t Motor_speed = 25;

// --- STABILITY CONTROL ---
const float KP_POS = 0.005;
const int8_t MIN_TORQUE = 18;
const int16_t FINAL_TOLERANCE = 100;
const long BRAKE_ZONE_TICKS = 1500;
const uint16_t MOVE_TIMEOUT_ITERATIONS = 5000;

// --- VECTORS ---
const int8_t V_FORWARD[]    = { 1, -1, -1,  1};
const int8_t V_BACKWARD[]   = {-1,  1,  1, -1};
const int8_t V_STRAFE_R[]   = { 1,  1,  1,  1};
const int8_t V_STRAFE_L[]   = {-1, -1, -1, -1};

const int8_t V_ROTATE_CW[]  = { 1, -1,  1, -1};
const int8_t V_ROTATE_CCW[] = {-1,  1, -1,  1};

const int8_t V_DIAG_FR[]    = { 1,  0,  0,  1};
const int8_t V_DIAG_FL[]    = { 0, -1, -1,  0};
const int8_t V_DIAG_BR[]    = { 0,  1,  1,  0};
const int8_t V_DIAG_BL[]    = {-1,  0,  0, -1};

// ================================================================
// ESP NOW FUNCTIONS
// ================================================================

void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status)
{
    Serial.print("ESP-NOW Send Status: ");

    if (status == ESP_NOW_SEND_SUCCESS)
        Serial.println("SUCCESS");
    else
        Serial.println("FAILED");
}

// ESP-NOW receive callback — handles pose replies from camera
void OnDataRecv(const esp_now_recv_info_t *recvInfo,
                const uint8_t *data, int len) {

    // Check if it's a PoseReply from camera
    if (len == sizeof(PoseReply)) {
        memcpy((void *)&lastPoseReply, data, sizeof(PoseReply));
        cameraPoseReceived = true;

        Serial.printf("[CAM] Pose: valid=%d color=%d conf=%.2f yaw=%.1f tz=%.1f est=%d\n",
                      lastPoseReply.pose_valid,
                      lastPoseReply.color,
                      lastPoseReply.confidence,
                      lastPoseReply.yaw_deg,
                      lastPoseReply.tz_mm,
                      lastPoseReply.estimated);
    }
}

void sendCommandToArm(const char* cmd) {

    strcpy(armMessage.command, cmd);

    esp_err_t result = esp_now_send(armAddress,
                                    (uint8_t *) &armMessage,
                                    sizeof(armMessage));

    Serial.print("Sending Command -> ");
    Serial.println(cmd);

    if (result == ESP_OK)
        Serial.println("Command Sent");
    else
        Serial.println("Error Sending Command");
}

void sendScanRequest(uint8_t mode) {
    ScanRequest req = {};
    static uint8_t taskCounter = 0;
    req.task_id = ++taskCounter;
    req.mode = mode;

    esp_err_t result = esp_now_send(cameraAddress,
                                    (uint8_t *)&req,
                                    sizeof(req));

    Serial.printf("[CAM] Scan request sent: task=%d mode=%d (%s)\n",
                  req.task_id, req.mode,
                  result == ESP_OK ? "OK" : "FAILED");
}

// Wait for a fresh camera PoseReply with timeout.
// Clears stale flag, sends scan request, polls until data arrives or timeout.
// Returns true if valid pose received.
static bool waitForCameraPose(unsigned long timeout_ms) {
    cameraPoseReceived = false;
    sendScanRequest(0); // mode=0: QR scan

    unsigned long t0 = millis();
    while (!cameraPoseReceived && millis() - t0 < timeout_ms) {
        Blynk.run();
        if (!autonomousMode) return false;
        delay(20);
    }

    if (!cameraPoseReceived) {
        Serial.println("[AUTO] Camera scan timeout");
        return false;
    }
    if (!lastPoseReply.pose_valid) {
        Serial.println("[AUTO] Camera reported invalid pose");
        return false;
    }
    return true;
}

// Camera-guided QR alignment for autonomous pickup.
// Uses camera yaw_deg to strafe until centered, then approaches until
// within APPROACH_DISTANCE_MM of the target, verifying confidence.
// Returns true if alignment succeeded and pose is usable.
static bool alignToQR() {
    const float YAW_THRESHOLD_DEG = 6.0f;
    const float APPROACH_DISTANCE_MM = 200.0f;
    const float CONFIDENCE_THRESHOLD = 0.55f;
    const int ALIGN_SPEED = 20;
    const int MAX_ALIGN_STEPS = 12;
    const int MAX_APPROACH_STEPS = 10;

    // Step 1: Initial scan
    if (!waitForCameraPose(5000)) return false;

    Serial.printf("[AUTO] Detected: color=%d conf=%.2f yaw=%.1f dist=%.0fmm\n",
                  lastPoseReply.color, lastPoseReply.confidence,
                  lastPoseReply.yaw_deg, lastPoseReply.tz_mm);

    // Step 2: Align yaw — strafe left/right until centered on QR
    for (int step = 0; step < MAX_ALIGN_STEPS; step++) {
        if (!autonomousMode) return false;
        if (fabs(lastPoseReply.yaw_deg) <= YAW_THRESHOLD_DEG) break;

        if (lastPoseReply.yaw_deg > 0)
            manualMove(V_STRAFE_R, ALIGN_SPEED);
        else
            manualMove(V_STRAFE_L, ALIGN_SPEED);

        delay(120);
        forceStop();

        if (!waitForCameraPose(3000)) return false;
    }
    forceStop();
    Serial.printf("[AUTO] Yaw aligned: %.1f°\n", lastPoseReply.yaw_deg);

    // Step 3: Approach — move forward until close enough
    for (int step = 0; step < MAX_APPROACH_STEPS; step++) {
        if (!autonomousMode) return false;
        if (lastPoseReply.tz_mm <= APPROACH_DISTANCE_MM) break;

        manualMove(V_FORWARD, ALIGN_SPEED);
        delay(200);
        forceStop();

        if (!waitForCameraPose(3000)) return false;
    }
    forceStop();
    Serial.printf("[AUTO] Approach complete: dist=%.0fmm conf=%.2f\n",
                  lastPoseReply.tz_mm, lastPoseReply.confidence);

    // Step 4: Verify confidence
    if (lastPoseReply.confidence < CONFIDENCE_THRESHOLD) {
        Serial.println("[AUTO] Low confidence, aborting pickup");
        return false;
    }

    return true;
}

// ================================================================
// HELPER FUNCTIONS
// ================================================================

bool writeBytes(uint8_t reg, uint8_t* data, size_t len) {
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(reg);
    for (size_t i = 0; i < len; i++) Wire.write(data[i]);
    return (Wire.endTransmission() == 0);
}

bool readEncoders(int32_t* data) {
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(REG_ENCODER_TOTAL);

    if (Wire.endTransmission() != 0) return false;
    if (Wire.requestFrom((uint8_t)I2C_ADDR, (uint8_t)16) != 16) return false;

    for (int i = 0; i < 4; i++) {
        uint32_t temp = 0;

        temp |= (uint32_t)Wire.read();
        temp |= (uint32_t)Wire.read() << 8;
        temp |= (uint32_t)Wire.read() << 16;
        temp |= (uint32_t)Wire.read() << 24;

        data[i] = (int32_t)temp;
    }

    return true;
}

void writeSpeeds(int8_t v1, int8_t v2, int8_t v3, int8_t v4) {
    int8_t speeds[4] = {v1, v2, v3, v4};
    writeBytes(REG_FIXED_SPEED, (uint8_t*)speeds, 4);
}

void manualMove(const int8_t vector[], int8_t speedVal) {

    writeSpeeds(
        vector[0] * speedVal,
        vector[1] * speedVal,
        vector[2] * speedVal,
        vector[3] * speedVal
    );
}

void forceStop() {
    writeSpeeds(0, 0, 0, 0);
}

// ================================================================
// MOVEMENT LOGIC
// ================================================================

void moveDistanceKp(const int8_t vector[], int8_t maxSpeed, float distance, float tickConstant) {

    int32_t startEncoders[4], currentEncoders[4];

    long targetTicks = lroundf(fabs(distance * tickConstant));

    int8_t localMinTorque = (tickConstant == TICKS_ROTATE) ? 22 : MIN_TORQUE;
    maxSpeed = max(maxSpeed, localMinTorque);

    if (!readEncoders(startEncoders)) return;

    long error = targetTicks;

    int8_t lastSpeed = -127;

    uint8_t loopCounter = 0;
    uint8_t i2cErrors = 0;

    while (true) {

        if (loopCounter % 2 == 0) {

            if (readEncoders(currentEncoders)) {
                i2cErrors = 0;
                double totalTraveled = 0;
                int activeMotors = 0;

                for(int i = 0; i < 4; i++) {

                    if (vector[i] != 0) {

                        int32_t diff = currentEncoders[i] - startEncoders[i];

                        totalTraveled += abs(diff);

                        activeMotors++;
                    }
                }

                long traveled = (long)(totalTraveled / (double)activeMotors);

                if (traveled >= targetTicks) break;

                error = targetTicks - traveled;
            } else {
                i2cErrors++;
                if (i2cErrors > 5) {
                    Serial.println("[ERR] I2C encoder read failed 5 times, aborting move");
                    break;
                }
            }
        }

        loopCounter++;

        if (error < FINAL_TOLERANCE) break;

        float calcSpeed = (float)error * KP_POS;

        int8_t finalSpeed;

        if (error < BRAKE_ZONE_TICKS)
            finalSpeed = localMinTorque;
        else
            finalSpeed = (int8_t)constrain(calcSpeed, localMinTorque, maxSpeed);

        if (finalSpeed != lastSpeed) {

            writeSpeeds(finalSpeed * vector[0],
                        finalSpeed * vector[1],
                        finalSpeed * vector[2],
                        finalSpeed * vector[3]);

            lastSpeed = finalSpeed;
        }

        if (loopCounter > MOVE_TIMEOUT_ITERATIONS) {
            Serial.println("[ERR] Move timed out (blocked or stalled), aborting");
            break;
        }

        delay(20);
    }

    forceStop();
}

void rotateDegrees(bool clockwise, float degrees, int8_t maxSpeed) {

    const int8_t* vector = clockwise ? V_ROTATE_CW : V_ROTATE_CCW;

    float distanceFraction = degrees / 360.0;

    moveDistanceKp(vector, maxSpeed, distanceFraction, TICKS_ROTATE);
}

// ===================== MANUAL CONTROL =====================

BLYNK_WRITE(V0) {
    if (!autonomousMode) {
        if (param.asInt()) {
            manualMove(V_FORWARD, Motor_speed);
            isMoving = true;
            lastMoveTime = millis();
            Serial.println("Going forward!");
        }
        else {
            forceStop();
            isMoving = false;
            Serial.println("stopping!");
        }
    }
}

BLYNK_WRITE(V1) {
    if (!autonomousMode) {
        if (param.asInt()) {
            manualMove(V_STRAFE_L, Motor_speed);
            isMoving = true;
            lastMoveTime = millis();
            Serial.println("Strafing left!");
        }
        else {
            forceStop();
            isMoving = false;
            Serial.println("stopping!");
        }
    }
}

BLYNK_WRITE(V2) {
    if (!autonomousMode) {
        if (param.asInt()) {
            manualMove(V_BACKWARD, Motor_speed);
            isMoving = true;
            lastMoveTime = millis();
            Serial.println("Going backward!");
        }
        else {
            forceStop();
            isMoving = false;
            Serial.println("stopping!");
        }
    }
}

BLYNK_WRITE(V3) {
    if (!autonomousMode) {
        if (param.asInt()) {
            manualMove(V_STRAFE_R, Motor_speed);
            isMoving = true;
            lastMoveTime = millis();
            Serial.println("Strafing right!");
        }
        else {
            forceStop();
            isMoving = false;
            Serial.println("stopping!");
        }
    }
}


BLYNK_WRITE(V4) {
    if (!autonomousMode) {
        if (param.asInt()) {
            manualMove(V_DIAG_FR, Motor_speed);
            isMoving = true;
            lastMoveTime = millis();
        }
        else {
            forceStop();
            isMoving = false;
            Serial.println("stopping!");
        }
    }
}

BLYNK_WRITE(V5) {
    if (!autonomousMode) {
        if (param.asInt()) {
            manualMove(V_DIAG_FL, Motor_speed);
            isMoving = true;
            lastMoveTime = millis();
        }
        else {
            forceStop();
            isMoving = false;
            Serial.println("stopping!");
        }
    }
}

BLYNK_WRITE(V6) {
    if (!autonomousMode) {
        if (param.asInt()) {
            manualMove(V_DIAG_BL, Motor_speed);
            isMoving = true;
            lastMoveTime = millis();
        }
        else {
            forceStop();
            isMoving = false;
        }
    }
}

BLYNK_WRITE(V7) {
    if (!autonomousMode) {
        if (param.asInt()) {
            manualMove(V_DIAG_BR, Motor_speed);
            isMoving = true;
            lastMoveTime = millis();
        }
        else {
            forceStop();
            isMoving = false;
            Serial.println("stopping!");
        }
    }
}

BLYNK_WRITE(V8) {
    if (!autonomousMode) {
        if (param.asInt()) {
            manualMove(V_ROTATE_CW, Motor_speed);
            isMoving = true;
            lastMoveTime = millis();
        }
        else {
            forceStop();
            isMoving = false;
            Serial.println("stopping!");
        }
    }
}

BLYNK_WRITE(V9) {
    if (!autonomousMode) {
        if (param.asInt()) {
            manualMove(V_ROTATE_CCW, Motor_speed);
            isMoving = true;
            lastMoveTime = millis();
        }
        else {
            forceStop();
            isMoving = false;
            Serial.println("stopping!");
        }
    }
}

// ===================== AUTO/MANUAL =====================

BLYNK_WRITE(V10) {

    if (param.asInt() && !autonomousMode) autoTrigger = 1;

    autonomousMode = param.asInt();

    forceStop();
    isMoving = false;

    if (autonomousMode)
        Serial.println("AUTONOMOUS MODE");
    else
        Serial.println("MANUAL MODE");
}

// ===================== ARM COMMANDS =====================

BLYNK_WRITE(V16) {
    if (param.asInt()){
        Serial.println("BTC");
        sendCommandToArm("BTC");    } 

}

BLYNK_WRITE(V17) {
    if (param.asInt()){
      sendCommandToArm("RTC");
      Serial.println("RTC");
    }

}

BLYNK_WRITE(V18) {
    if (param.asInt()){
       sendCommandToArm("GTC");
       Serial.println("GTC");
}
}
BLYNK_WRITE(V19) {
    if (param.asInt()) {
      sendCommandToArm("H");
     Serial.println("H");
    }

}

BLYNK_WRITE(V20) {
    
        if (param.asInt()) {
    sendCommandToArm("RTF");
     Serial.println("RTF");
    }
}

BLYNK_WRITE(V24) {
    if (param.asInt()) {
      sendCommandToArm("BTF");
      Serial.println("BTF");
    }
}

BLYNK_WRITE(V25) {
    int raw = param.asInt();
    // Constrain to int8_t safe range [0, 100] to prevent overflow
    Motor_speed = (int8_t)constrain(raw, 0, 100);
    Serial.print("Motor Speed Updated: ");
    Serial.println(Motor_speed);
    Blynk.virtualWrite(V23, Motor_speed);
}

// ===================== NEW ARM COMMANDS =====================

// V15 — Green To Floor
BLYNK_WRITE(V15) {
    if (param.asInt()) {
        sendCommandToArm("GTF");
        Serial.println("GTF");
    }
}

// V27 — From Car To Rod
BLYNK_WRITE(V27) {
    if (param.asInt()) {
        sendCommandToArm("FCTR");
        Serial.println("FCTR");
    }
}

// V28 — Servo Index selector (slider 0-4, stores value for V30)
int selectedServo = 0;
BLYNK_WRITE(V28) {
    selectedServo = constrain(param.asInt(), 0, 4);
    Serial.printf("[SERVO] Selected servo: %d\n", selectedServo);
}

// V29 — Servo Angle selector (slider 0-180, stores value for V30)
int selectedAngle = 90;
BLYNK_WRITE(V29) {
    selectedAngle = constrain(param.asInt(), 0, 180);
    Serial.printf("[SERVO] Selected angle: %d\n", selectedAngle);
}

// V30 — Send Servo Move command (button)
BLYNK_WRITE(V30) {
    if (param.asInt()) {
        // Format: "SV:index:angle" e.g. "SV:0:90"
        char cmd[10];
        snprintf(cmd, sizeof(cmd), "SV:%d:%d", selectedServo, selectedAngle);
        sendCommandToArm(cmd);
        Serial.printf("[SERVO] Sent: %s\n", cmd);
    }
}

// ===================== CAMERA SCAN COMMANDS =====================

// V21 — Trigger QR scan (ask camera to detect QR codes)
BLYNK_WRITE(V21) {
    if (param.asInt()) {
        sendScanRequest(0); // mode=0: QR scan
        Serial.println("[SCAN] QR scan requested");
    }
}

// V22 — Trigger platform scan (ask camera to detect square platform)
BLYNK_WRITE(V22) {
    if (param.asInt()) {
        sendScanRequest(1); // mode=1: platform scan
        Serial.println("[SCAN] Platform scan requested");
    }
}

// V26 — Move arm to scan position (camera looks forward-down)
BLYNK_WRITE(V26) {
    if (param.asInt()) {
        sendCommandToArm("S");
        Serial.println("[SCAN] Arm moving to scan pose");
    }
}

// ================================================================
// SETUP
// ================================================================

void setup() {

    Serial.begin(115200);

    Wire.begin(SDA_PIN, SCL_PIN, 40000);

    delay(500);

    uint8_t motorType = 3;

    writeBytes(REG_MOTOR_TYPE, &motorType, 1);

    delay(10);

    uint8_t polarity = 0;

    writeBytes(REG_MOTOR_PHASE, &polarity, 1);

    delay(10);

    uint8_t resetData[16] = {0};

    writeBytes(REG_ENCODER_TOTAL, resetData, 16);

    forceStop();

    Serial.println("System Ready");

    // ================= WIFI =================
    WiFi.mode(WIFI_STA);

    WiFi.begin(ssid, pass);

    Serial.print("Connecting WiFi");

    while (WiFi.status() != WL_CONNECTED) {

        delay(500);

        Serial.print(".");
    }

    Serial.println("\nWiFi Connected");

    Serial.print("BASE MAC: ");
    Serial.println(WiFi.macAddress());

    Serial.print("BASE CHANNEL: ");
    Serial.println(WiFi.channel());

    // ================= BLYNK (Local Server) =================
    Blynk.config(auth, blynkServer, BLYNK_LOCAL_PORT);

    if (!Blynk.connect(5000)) {
        Serial.println("Blynk LOCAL SERVER connection failed!");
    } else {
        Serial.println("Blynk LOCAL SERVER connected!");
    }

    // ================= ESP NOW =================
    if (esp_now_init() != ESP_OK) {

        Serial.println("ESP-NOW INIT FAILED");

        return;
    }

    esp_now_register_send_cb(OnDataSent);

    esp_now_peer_info_t peerInfo = {};

    memcpy(peerInfo.peer_addr, armAddress, 6);

    peerInfo.channel = WiFi.channel();

    peerInfo.encrypt = false;

    peerInfo.ifidx = WIFI_IF_STA;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {

        Serial.println("Failed to Add Peer");

        return;
    }

    Serial.println("ESP-NOW READY");

    // ================= CAMERA ESP-NOW PEER =================
    esp_now_register_recv_cb(OnDataRecv);

    esp_now_peer_info_t camPeer = {};
    memcpy(camPeer.peer_addr, cameraAddress, 6);
    camPeer.channel = WiFi.channel();
    camPeer.encrypt = false;
    camPeer.ifidx = WIFI_IF_STA;

    if (esp_now_add_peer(&camPeer) != ESP_OK) {
        Serial.println("Failed to Add Camera Peer");
    } else {
        Serial.println("Camera ESP-NOW Peer Added");
    }
}

void loop() {

    Blynk.run();

    // =================== MOVEMENT WATCHDOG ===================
    // Safety net: auto-stop if movement active for longer than timeout
    // Catches missed Blynk button release events
    if (isMoving && (millis() - lastMoveTime > MOVE_TIMEOUT_MS)) {
        forceStop();
        isMoving = false;
        Serial.println("[WATCHDOG] Auto-stop: move timeout");
    }

    // Push latest camera data to Blynk every 2 seconds
    static unsigned long lastCamUpdate = 0;
    if (cameraPoseReceived && millis() - lastCamUpdate > 2000) {
        lastCamUpdate = millis();

        // V11 = confidence (0-100 for gauge widget)
        Blynk.virtualWrite(V11, (int)(lastPoseReply.confidence * 100));

        // V12 = yaw angle (for strafing indicator)
        Blynk.virtualWrite(V12, lastPoseReply.yaw_deg);

        // V13 = detected color text
        const char* colorNames[] = {"NONE", "RED", "GREEN", "BLUE"};
        int ci = lastPoseReply.color;
        if (ci > 3) ci = 0;
        Blynk.virtualWrite(V13, colorNames[ci]);

        // V14 = distance in mm
        Blynk.virtualWrite(V14, lastPoseReply.tz_mm);
    }

    if (autonomousMode && autoTrigger == 1)
    {
        Serial.println("[AUTO] Starting camera-guided autonomous pickup");

        // Targets in order: RED, GREEN, BLUE
        struct {
            uint8_t colorCode;
            const char *armCmd;
            const char *label;
        } const targets[] = {
            {ARM_COLOR_R, "RTF", "RED"},
            {ARM_COLOR_G, "GTF", "GREEN"},
            {ARM_COLOR_B, "BTF", "BLUE"},
        };

        for (int i = 0; i < 3; i++) {
            if (!autonomousMode) break;
            Serial.printf("[AUTO] --- Target %d: %s ---\n", i + 1, targets[i].label);

            if (!alignToQR()) {
                Serial.printf("[AUTO] Alignment failed for %s, skipping\n", targets[i].label);
                continue;
            }

            if (!autonomousMode) break;

            // Check detected color matches expected target
            if (lastPoseReply.color == targets[i].colorCode) {
                Serial.printf("[AUTO] Color match: %s — sending arm command\n", targets[i].label);
                sendCommandToArm(targets[i].armCmd);
                
                unsigned long t0 = millis();
                while (millis() - t0 < 9000) {
                    Blynk.run();
                    if (!autonomousMode) break;
                    delay(50);
                }
            } else {
                const char *colorNames[] = {"NONE", "RED", "GREEN", "BLUE"};
                int ci = lastPoseReply.color;
                if (ci > 3) ci = 0;
                Serial.printf("[AUTO] Color mismatch: expected %s, camera sees %s — skipping\n",
                              targets[i].label, colorNames[ci]);
            }
        }

        if (autonomousMode) sendCommandToArm("H");
        forceStop();
        autoTrigger = 0;
        Serial.println("[AUTO] Autonomous pickup complete");
    }
}