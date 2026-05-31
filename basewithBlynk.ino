// =========================== BASE ESP32 CODE ===========================

// ================= LOCAL BLYNK SERVER =================
// Replace 192.168.X.X with your computer's local IP address
#define BLYNK_PRINT Serial
#define BLYNK_LOCAL_PORT 8080

bool flag = 0;

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

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
// Camera ESP32-S3 MAC Address — update when you read it from Serial Monitor
// Camera prints "CAMERA MAC: xx:xx:xx:xx:xx:xx" on boot
static uint8_t cameraAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // CHANGE THIS

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
static volatile bool cameraPoseReceived = false;
static PoseReply lastPoseReply = {};

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
    for (int i = 0; i < 5; i++) {
        writeSpeeds(0, 0, 0, 0);
        delay(10);
    }
}

// ================================================================
// MOVEMENT LOGIC
// ================================================================

void moveDistanceKp(const int8_t vector[], int8_t maxSpeed, float distance, float tickConstant) {

    int32_t startEncoders[4], currentEncoders[4];

    long targetTicks = (long)abs(distance * tickConstant);

    int8_t localMinTorque = (tickConstant == TICKS_ROTATE) ? 22 : MIN_TORQUE;

    if (!readEncoders(startEncoders)) return;

    long error = targetTicks;

    int8_t lastSpeed = -127;

    uint8_t loopCounter = 0;

    while (true) {

        if (loopCounter % 2 == 0) {

            if (readEncoders(currentEncoders)) {

                double totalTraveled = 0;
                int activeMotors = 0;

                for(int i = 0; i < 4; i++) {

                    if (vector[i] != 0) {

                        long diff = (long)currentEncoders[i] - (long)startEncoders[i];

                        totalTraveled += abs(diff);

                        activeMotors++;
                    }
                }

                long traveled = (long)(totalTraveled / (double)activeMotors);

                if (traveled >= targetTicks) break;

                error = targetTicks - traveled;
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

    autonomousMode = param.asInt();

    flag = 1;

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
    Motor_speed = param.asInt();  // slider value from Blynk

    Serial.print("Motor Speed Updated: ");
    Serial.println(Motor_speed);
        // reflect immediately on gauge
    Blynk.virtualWrite(V23, Motor_speed);
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

    if (autonomousMode && flag == 1)
    {
        while (autonomousMode) {

            moveDistanceKp(V_STRAFE_R, Motor_speed, 1.0, TICKS_STRAFE);
            sendCommandToArm("RTF");
            delay(9000);

            moveDistanceKp(V_FORWARD, Motor_speed, 1.0, TICKS_FWD_BWD);
            sendCommandToArm("GTF");
            delay(9000);

            moveDistanceKp(V_DIAG_BL, Motor_speed, 0.70, TICKS_DIAG);
            sendCommandToArm("BTF");
            delay(9000);

            flag = 0;

            break;
        }
        delay(200);
        sendCommandToArm("H");
        forceStop();
    }
}