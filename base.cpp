// =========================== BASE ESP32 (WiFi AP + HTTP + ESP-NOW)
// =========================== Phone → WiFi "GLITCH" → http://192.168.4.1 → HTTP
// POST → ESP-NOW → Arm/Camera

#include "MPU6050_6Axis_MotionApps20.h"
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>

// ================= WIFI AP =================
const char *AP_SSID = "GLITCH";
const char *AP_PASS = "Gl1tch2024!Secure";
const uint8_t WIFI_CHANNEL = 11;

// ================= WEB SERVER =================
WebServer server(80);

// ================= WEBSOCKET SERVER (fallback for ESP-NOW) =================
static const uint16_t WS_PORT = 8080;
static WebSocketsServer wsServer(WS_PORT);

static void wsArmSendCommand(const char *cmd) {
  // Send command to arm via WebSocket (binary, same struct as ESP-NOW)
  struct {
    char command[10];
  } armMsg;
  strncpy(armMsg.command, cmd, 9);
  armMsg.command[9] = '\0';
  wsServer.sendBIN(0, (uint8_t *)&armMsg, sizeof(armMsg));
  Serial.printf("[WS] Sent to arm: %s\n", cmd);
}

static void wsArmSendBIN(const uint8_t *data, size_t len) {
  wsServer.sendBIN(0, data, len);
}

// ================= DEAD RECKONING (ENCODER-BASED) =================
float posX = 0, posY = 0; // global position in mm
float velX = 0, velY = 0; // filtered velocity in mm/s
float currentSpeed = 0;   // filtered scalar speed mm/s
float speedKmh = 0;       // filtered speed in km/h
int32_t lastEncoders[4] = {0, 0, 0, 0};
unsigned long lastEncoderMs = 0;
bool encodersInitialized = false;
// 1980 ticks/rev, 80mm wheel diameter
const float TICKS_PER_REV = 1980.0f;
const float WHEEL_DIA_MM = 80.0f;
const float MM_PER_TICK =
    (PI * WHEEL_DIA_MM) / TICKS_PER_REV; // ~0.1269 mm/tick
// EMA filter (0 < alpha < 1, lower = smoother)
const float EMA_ALPHA = 0.15f;

// ================= ESP-NOW SHARED ENUMS =================
enum ArmColorCode : uint8_t {
  ARM_COLOR_UNKNOWN = 0,
  ARM_COLOR_R = 1,
  ARM_COLOR_G = 2,
  ARM_COLOR_B = 3,
};

// ================= ESP NOW =================

// Camera ESP32 MAC Address

// =================== CAMERA ESP-NOW ===================
static uint8_t cameraAddress[] = {
    0x94, 0xA9, 0x90, 0x08, 0xB2, 0xB8}; // must match camera's WiFi STA MAC

// ESP-NOW packet types (must match camera firmware)
enum EspNowPacketType : uint8_t {
  ESPNOW_TYPE_SCAN_REQ = 0x20,
  ESPNOW_TYPE_POSE_REPLY = 0x30,
};

// ESP-NOW packet: Base → Camera (scan request)
struct __attribute__((packed)) ScanRequest {
  uint8_t type; // ESPNOW_TYPE_SCAN_REQ (0x20)
  uint8_t task_id;
  uint8_t mode; // 0=scan_qr, 1=scan_platform
  uint8_t reserved;
};

// ESP-NOW packet: Camera → Base (pose reply)
struct __attribute__((packed)) PoseReply {
  uint8_t type; // ESPNOW_TYPE_POSE_REPLY (0x30)
  uint8_t task_id;
  uint8_t pose_valid;
  uint8_t color; // 0=unknown, 1=R, 2=G, 3=B
  uint8_t estimated;
  float tx_mm;
  float ty_mm;
  float tz_mm;
  float yaw_deg;
  float confidence;
  uint8_t status; // 0=Accumulating, 1=DONE
};

// ESP-NOW packet: Base → Arm (camera pose data for guided IK)
struct __attribute__((packed)) CameraPoseData {
  uint8_t type; // 1 = camera pose packet
  uint8_t pose_valid;
  uint8_t color;
  uint8_t estimated;
  float tx_mm;
  float ty_mm;
  float tz_mm;
  float yaw_deg;
  float confidence;
};

// ESP-NOW packet: Arm → Base (busy/idle status)
struct __attribute__((packed)) ArmStatus {
  uint8_t type; // 0 = status
  uint8_t busy; // 1 = busy, 0 = idle
  uint8_t pad[2];
};

// Latest camera pose data (updated by ESP-NOW callback)
static volatile bool cameraPoseReceived = false;
static volatile PoseReply lastPoseReply;

// Latest arm status (updated by ESP-NOW callback)
static volatile bool armStatusReceived = false;
static volatile ArmStatus lastArmStatus;

// WebSocket event handler — receives arm status as fallback
void onWsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_CONNECTED) {
    Serial.printf("[WS] Arm connected: %d\n", num);
  } else if (type == WStype_DISCONNECTED) {
    Serial.printf("[WS] Arm disconnected: %d\n", num);
  } else if (type == WStype_BIN && length >= 4) {
    uint8_t pktType = payload[0];
    if (pktType == 0 && length >= 4) {
      memcpy((void *)&lastArmStatus, payload, sizeof(ArmStatus));
      armStatusReceived = true;
      Serial.printf("[WS-ARM] busy=%d\n", lastArmStatus.busy);
    }
  }
}

// ESP-NOW recv debug counter
static volatile uint32_t espNowRecvCount = 0;
static volatile uint32_t espNowRecvBytes = 0;
static volatile uint32_t espNowRecvMatchPose = 0;

// Scan progress tracking
static volatile bool scanInProgress = false;
static volatile uint8_t lastScanStatus = 0; // 0=Accumulating, 1=DONE
static unsigned long scanStartMs = 0;
static portMUX_TYPE camMux = portMUX_INITIALIZER_UNLOCKED;

// Last QR scan result (persists until new scan)
static bool lastQrValid = false;
static uint8_t lastQrColor = 0;
static float lastQrTx = 0, lastQrTy = 0, lastQrTz = 0, lastQrYaw = 0,
             lastQrConf = 0;
static bool newQrResult = false;

// --- STEP MOVEMENT FLAG ---
bool pendingStep = false;
String stepArg = "";

// --- CONTINUOUS MOVEMENT STATE ---
bool moveActive = false;
const int8_t *moveVec = nullptr;
int8_t moveSpeed = 0;
float moveTargetHeading = 0;
bool moveIsRotation = false;

// --- HEADING PID (gyro correction during linear movement) ---
const float KP_HEADING = 0.5f;
const float KI_HEADING = 0.12f;
const float KD_HEADING = 1.0f;
const float I_MAX_HEADING = 15.0f;
const float HEADING_DEADBAND = 2.0f;
static float headingIntegral = 0.0f;
static float headingPrevError = 0.0f;

// --- ROBOT STATE ---
String robotState = "Idle";

const char *stateFromVector(const int8_t vec[]) {
  if (vec[0] == 1 && vec[1] == -1 && vec[2] == -1 && vec[3] == 1)
    return "Moving Forward";
  if (vec[0] == -1 && vec[1] == 1 && vec[2] == 1 && vec[3] == -1)
    return "Moving Backward";
  if (vec[0] == -1 && vec[1] == -1 && vec[2] == -1 && vec[3] == -1)
    return "Strafing Left";
  if (vec[0] == 1 && vec[1] == 1 && vec[2] == 1 && vec[3] == 1)
    return "Strafing Right";
  if (vec[0] == 1 && vec[1] == -1 && vec[2] == 1 && vec[3] == -1)
    return "Rotating CW";
  if (vec[0] == -1 && vec[1] == 1 && vec[2] == -1 && vec[3] == 1)
    return "Rotating CCW";
  return "Moving Diagonal";
}

// --- I2C Registers ---
#define I2C_ADDR 0x34
#define REG_MOTOR_TYPE 0x14
#define REG_MOTOR_PHASE 0x15
#define REG_FIXED_SPEED 0x33
#define REG_ENCODER_TOTAL 0x3C

// --- Pins ---
#define SDA_PIN 21
#define SCL_PIN 22

// --- I2C Diagnostics ---
static uint32_t i2cWriteFails = 0;
static uint32_t i2cReadFails = 0;

bool autonomousMode = false;
int autoTrigger = 0;

// --- MPU6050 DMP ---
MPU6050 mpu;
bool dmpReady = false;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
float currentYaw = 0;
float targetHeading = 0;
void updateYaw();
void updateDeadReckoning();
void trackMovement(const int8_t vector[], float dist_mm);

// --- CALIBRATED TICK CONSTANTS (Blynk values × 97/80, distance in meters) ---
const float TICKS_FWD_BWD = 7463.611f; // updated by 100/90 factor
const float TICKS_STRAFE = 7897.61f;
const float TICKS_DIAG = 9548.44f;
const float TICKS_ROTATE = 8730.00f;

// --- COMPETITION DISTANCES (meters) - update after calibration ---
const float DIST_1 = 0.615; // FWD to red box
const float DIST_2 = 1.33;  // FWD to platform
const float DIST_3 = 0.30;  // BWD centering //was 0.5
const float DIST_4 = 1.65;  // RIGHT to green box
const float DIST_5 = 0.1;   // LEFT centering
const float DIST_6 = 1.45;  // FWD after 180 turn
const float DIST_7 = 0.9;   // DIAG to blue box

int8_t Motor_speed = 100;

// --- STABILITY CONTROL ---
const float KP_POS = 0.005;
const float KP_GYRO = 2.5;
const int8_t MIN_TORQUE = 18;
const int16_t FINAL_TOLERANCE = 100;
const long BRAKE_ZONE_TICKS = 1500;
const uint16_t MOVE_TIMEOUT_ITERATIONS = 5000;

// --- VECTORS ---
const int8_t V_FORWARD[] = {1, -1, -1, 1};
const int8_t V_BACKWARD[] = {-1, 1, 1, -1};
const int8_t V_STRAFE_R[] = {1, 1, 1, 1};
const int8_t V_STRAFE_L[] = {-1, -1, -1, -1};

// --- MECHANICAL COMPENSATION ---
// Right strafe drifts forward slightly; subtract from front, add to back
const int8_t STRAFE_R_BACK_COMP = 20; // % of motor speed
// Left strafe drifts backward slightly; add forward, subtract back
const int8_t STRAFE_L_FWD_COMP = 20; // % of motor speed

const int8_t V_ROTATE_CW[] = {1, -1, 1, -1};
const int8_t V_ROTATE_CCW[] = {-1, 1, -1, 1};

const int8_t V_DIAG_FR[] = {1, 0, 0, 1};
const int8_t V_DIAG_FL[] = {0, -1, -1, 0};
const int8_t V_DIAG_BR[] = {0, 1, 1, 0};
const int8_t V_DIAG_BL[] = {-1, 0, 0, -1};

// --- SERVO STEP CONTROL ---
const int SERVO_STEP_DEG[4] = {5, 3, 3, 2};
static float servoAngle[4] = {90.0f, 170.0f, 180.0f, 100.0f};

// ================================================================
// ESP NOW FUNCTIONS
// ================================================================

// ESP-NOW send callback
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("[ESP-NOW] Send SUCCESS");
  } else {
    Serial.println("[ESP-NOW] Send FAILED");
  }
}

// ESP-NOW receive callback — JUST STORE DATA, no heavy operations
// Runs in WiFi task — MUST return immediately.
void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (!mac || !data || len < 1)
    return;
  bool fromCam = (memcmp(mac, cameraAddress, 6) == 0);
  if (!fromCam)
    return;

  espNowRecvCount++;
  espNowRecvBytes += len;
  if (len == (int)sizeof(PoseReply) && data[0] == ESPNOW_TYPE_POSE_REPLY) {
    portENTER_CRITICAL(&camMux);
    memcpy((void *)&lastPoseReply, data, sizeof(PoseReply));
    portEXIT_CRITICAL(&camMux);
    cameraPoseReceived = true;
    lastScanStatus = lastPoseReply.status;
    if (lastPoseReply.status == 1)
      scanInProgress = false;
    espNowRecvMatchPose++;
  }
}

void sendCommandToArm(const char *cmd) {
  wsArmSendCommand(cmd);
  Serial.printf("[WS] Send to arm: %s\n", cmd);
}

void sendScanRequest(uint8_t mode) {
  ScanRequest req = {};
  static uint8_t taskCounter = 0;
  req.type = ESPNOW_TYPE_SCAN_REQ;
  req.task_id = ++taskCounter;
  req.mode = mode;
  esp_err_t result = esp_now_send(cameraAddress, (uint8_t *)&req, sizeof(req));
  if (result == ESP_OK) {
    scanInProgress = true;
    lastScanStatus = 0;
    scanStartMs = millis();
    Serial.printf("[CAM] Scan request sent: task=%d mode=%d\n", req.task_id,
                  req.mode);
  } else {
    scanInProgress = false;
    Serial.printf("[CAM] Scan request FAILED: task=%d mode=%d err=%d\n",
                  req.task_id, req.mode, result);
  }
}

void sendCameraPoseToArm() {
  CameraPoseData data = {};
  data.type = 1;
  portENTER_CRITICAL(&camMux);
  data.pose_valid = lastPoseReply.pose_valid;
  data.color = lastPoseReply.color;
  data.estimated = lastPoseReply.estimated;
  data.tx_mm = lastPoseReply.tx_mm;
  data.ty_mm = lastPoseReply.ty_mm;
  data.tz_mm = lastPoseReply.tz_mm;
  data.yaw_deg = lastPoseReply.yaw_deg;
  data.confidence = lastPoseReply.confidence;
  portEXIT_CRITICAL(&camMux);

  wsArmSendBIN((uint8_t *)&data, sizeof(data));
  Serial.printf("[WS] Camera pose forwarded to arm: valid=%d color=%d "
                "tx=%.0f ty=%.0f tz=%.0f yaw=%.1f conf=%.2f\n",
                data.pose_valid, data.color, data.tx_mm, data.ty_mm, data.tz_mm,
                data.yaw_deg, data.confidence);
}

// ================================================================
// I2C HELPERS
// ================================================================

static void i2cBusRecover() {
  Serial.println("[I2C] Bus recovery");
  Wire.end();
  pinMode(SCL_PIN, OUTPUT);
  for (int i = 0; i < 16; i++) {
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(5);
  }
  pinMode(SCL_PIN, INPUT);
  Wire.begin(SDA_PIN, SCL_PIN, 100000);
  Wire.setTimeOut(20);
  delay(5);
}

bool writeBytes(uint8_t reg, uint8_t *data, size_t len) {
  for (int attempt = 0; attempt < 3; attempt++) {
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(reg);
    for (size_t i = 0; i < len; i++)
      Wire.write(data[i]);
    if (Wire.endTransmission() == 0)
      return true;
    delay(1);
  }
  i2cWriteFails++;
  if (i2cWriteFails % 20 == 1)
    Serial.printf("[I2C] writeBytes failed %lu times\n",
                  (unsigned long)i2cWriteFails);
  i2cBusRecover();
  return false;
}

bool readEncoders(int32_t *data) {
  for (int attempt = 0; attempt < 3; attempt++) {
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(REG_ENCODER_TOTAL);
    if (Wire.endTransmission() != 0) {
      delay(1);
      continue;
    }
    if (Wire.requestFrom((uint8_t)I2C_ADDR, (uint8_t)16) != 16) {
      delay(1);
      continue;
    }
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
  i2cReadFails++;
  if (i2cReadFails % 20 == 1)
    Serial.printf("[I2C] readEncoders failed %lu times\n",
                  (unsigned long)i2cReadFails);
  i2cBusRecover();
  return false;
}

// --- Motor speed cache (skip identical I2C writes) ---
static int8_t lastWrittenSpeeds[4] = {-128, -128, -128, -128};

static uint32_t lastSpeedWriteMs = 0;
#define SPEED_REFRESH_MS 100

bool writeSpeeds(int8_t v1, int8_t v2, int8_t v3, int8_t v4) {
  uint32_t now = millis();
  bool speedsChanged =
      (v1 != lastWrittenSpeeds[0] || v2 != lastWrittenSpeeds[1] ||
       v3 != lastWrittenSpeeds[2] || v4 != lastWrittenSpeeds[3]);
  bool refreshDue = (now - lastSpeedWriteMs >= SPEED_REFRESH_MS);

  if (!speedsChanged && !refreshDue)
    return true;

  int8_t speeds[4] = {v1, v2, v3, v4};
  if (writeBytes(REG_FIXED_SPEED, (uint8_t *)speeds, 4)) {
    lastWrittenSpeeds[0] = v1;
    lastWrittenSpeeds[1] = v2;
    lastWrittenSpeeds[2] = v3;
    lastWrittenSpeeds[3] = v4;
    lastSpeedWriteMs = now;
    return true;
  }
  return false;
}

static uint32_t lastMotorDbgMs = 0;

void updateMotors() {
  if (!moveActive)
    return;
  robotState = stateFromVector(moveVec);

  int8_t fl = moveSpeed * moveVec[0];
  int8_t fr = moveSpeed * moveVec[1];
  int8_t bl = moveSpeed * moveVec[2];
  int8_t br = moveSpeed * moveVec[3];

  // Strafe right mechanical compensation: subtract from front, add to back
  if (moveVec[0] == 1 && moveVec[1] == 1 && moveVec[2] == 1 &&
      moveVec[3] == 1) {
    int8_t comp = (int8_t)((int)moveSpeed * STRAFE_R_BACK_COMP / 100);
    fl -= comp;
    fr -= comp;
    bl += comp;
    br += comp;
  }

  // Strafe left mechanical compensation: add forward, subtract back
  if (moveVec[0] == -1 && moveVec[1] == -1 && moveVec[2] == -1 &&
      moveVec[3] == -1) {
    int8_t comp = (int8_t)((int)moveSpeed * STRAFE_L_FWD_COMP / 100);
    fl += comp;
    fr -= comp;
    bl -= comp;
    br += comp;
  }

  if (!moveIsRotation) {
    float yawError = moveTargetHeading - currentYaw;
    if (yawError > 180)
      yawError -= 360;
    if (yawError < -180)
      yawError += 360;
    if (fabs(yawError) > HEADING_DEADBAND) {
      if (fabs(yawError) < 10.0f)
        headingIntegral += yawError;
      headingIntegral =
          constrain(headingIntegral, -I_MAX_HEADING, I_MAX_HEADING);
      float dError = yawError - headingPrevError;
      float gyroCorr = KP_HEADING * yawError + KI_HEADING * headingIntegral +
                       KD_HEADING * dError;
      int8_t corr = (int8_t)constrain(gyroCorr, -127.0f, 127.0f);
      fl += corr;
      fr -= corr;
      bl += corr;
      br -= corr;
    }
    headingPrevError = yawError;
  }

  int8_t ofl = constrain(fl, -100, 100);
  int8_t ofr = constrain(fr, -100, 100);
  int8_t obl = constrain(bl, -100, 100);
  int8_t obr = constrain(br, -100, 100);

  if (millis() - lastMotorDbgMs > 500) {
    Serial.printf("[MOT] spd=%d vec=[%d,%d,%d,%d] yaw=%.1f tgt=%.1f fl=%d "
                  "fr=%d bl=%d br=%d\n",
                  (int)moveSpeed, (int)moveVec[0], (int)moveVec[1],
                  (int)moveVec[2], (int)moveVec[3], currentYaw,
                  moveTargetHeading, (int)ofl, (int)ofr, (int)obl, (int)obr);
    lastMotorDbgMs = millis();
  }

  writeSpeeds(ofl, ofr, obl, obr);
}

void forceStop() {
  moveActive = false;
  robotState = "Idle";
  lastWrittenSpeeds[0] = -128;
  for (int i = 0; i < 5; i++) {
    if (writeSpeeds(0, 0, 0, 0)) {
      break;
    }
    delay(5);
  }
}

// ================================================================
// MPU6050 DMP
// ================================================================

void updateYaw() {
  if (dmpReady) {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      currentYaw = (ypr[0] * 180.0f / M_PI);
    }
  }
}

// ================================================================
// MOVEMENT LOGIC
// ================================================================

void moveDistanceKp(const int8_t vector[], int8_t maxSpeed, float distance,
                    float tickConstant) {
  robotState = stateFromVector(vector);
  int32_t startEncoders[4], currentEncoders[4];
  long targetTicks = lroundf(fabs(distance * tickConstant));
  int8_t localMinTorque = MIN_TORQUE;
  maxSpeed = max(maxSpeed, localMinTorque);

  updateYaw();
  targetHeading = currentYaw;
  headingIntegral = 0.0f;
  headingPrevError = 0.0f;

  if (!readEncoders(startEncoders))
    return;
  long error = targetTicks;
  uint8_t loopCounter = 0;
  uint8_t i2cErrors = 0;

  while (true) {
    server.handleClient();
    wsServer.loop();
    updateYaw();

    if (loopCounter % 2 == 0) {
      if (readEncoders(currentEncoders)) {
        i2cErrors = 0;
        double totalTraveled = 0;
        int activeMotors = 0;
        for (int i = 0; i < 4; i++) {
          if (vector[i] != 0) {
            int32_t diff = currentEncoders[i] - startEncoders[i];
            totalTraveled += abs(diff);
            activeMotors++;
          }
        }
        long traveled = (long)(totalTraveled / (double)activeMotors);
        if (traveled >= targetTicks)
          break;
        error = targetTicks - traveled;
      } else {
        i2cErrors++;
        if (i2cErrors > 5) {
          Serial.println(
              "[ERR] I2C encoder read failed 5 times, aborting move");
          break;
        }
      }
    }
    loopCounter++;
    if (error < FINAL_TOLERANCE)
      break;
    float calcSpeed = (float)error * KP_POS;
    int8_t baseSpeed;
    if (error < BRAKE_ZONE_TICKS)
      baseSpeed = localMinTorque;
    else
      baseSpeed = (int8_t)constrain(calcSpeed, localMinTorque, maxSpeed);

    float yawError = targetHeading - currentYaw;
    if (yawError > 180)
      yawError -= 360;
    if (yawError < -180)
      yawError += 360;
    if (fabs(yawError) < 10.0f)
      headingIntegral += yawError;
    headingIntegral = constrain(headingIntegral, -I_MAX_HEADING, I_MAX_HEADING);
    float dError = yawError - headingPrevError;
    float gyroCorr = KP_HEADING * yawError + KI_HEADING * headingIntegral +
                     KD_HEADING * dError;
    int8_t corr = (int8_t)constrain(gyroCorr, -127.0f, 127.0f);
    headingPrevError = yawError;

    writeSpeeds(constrain((int)(baseSpeed * vector[0]) + corr, -100, 100),
                constrain((int)(baseSpeed * vector[1]) - corr, -100, 100),
                constrain((int)(baseSpeed * vector[2]) + corr, -100, 100),
                constrain((int)(baseSpeed * vector[3]) - corr, -100, 100));

    if (loopCounter > MOVE_TIMEOUT_ITERATIONS) {
      Serial.println("[ERR] Move timed out (blocked or stalled), aborting");
      break;
    }
    updateDeadReckoning();
    delay(15);
  }
  forceStop();
}

void rotateDegrees(bool clockwise, float degrees, int8_t maxSpeed) {
  robotState = clockwise ? "Rotating CW" : "Rotating CCW";

  const float KP_ROTATE = 0.5f;
  const float KI_ROTATE = 0.12f;
  const float KD_ROTATE = 1.0f;
  const float MIN_SPEED = 12.0f;
  const float STOP_THRESHOLD = 1.0f;
  const float I_MAX = 15.0f;
  const float ROT_D_FILTER_ALPHA = 0.3f;

  updateYaw();
  float startHeading = currentYaw;
  float target =
      clockwise ? (startHeading + degrees) : (startHeading - degrees);

  float prevError = degrees;
  float integral = 0.0f;
  float rotFilteredDError = 0.0f;

  while (true) {
    server.handleClient();
    wsServer.loop();
    updateYaw();

    float yawError = target - currentYaw;
    if (yawError > 180)
      yawError -= 360;
    if (yawError < -180)
      yawError += 360;

    if (fabs(yawError) < STOP_THRESHOLD)
      break;

    if (fabs(yawError) < 10.0f)
      integral += yawError;
    integral = constrain(integral, -I_MAX, I_MAX);

    float dError = yawError - prevError;
    rotFilteredDError = ROT_D_FILTER_ALPHA * dError +
                        (1.0f - ROT_D_FILTER_ALPHA) * rotFilteredDError;
    float turnSpeed = KP_ROTATE * yawError + KI_ROTATE * integral +
                      KD_ROTATE * rotFilteredDError;

    if (fabs(turnSpeed) < MIN_SPEED && fabs(yawError) > STOP_THRESHOLD * 3) {
      turnSpeed = (turnSpeed > 0) ? MIN_SPEED : -MIN_SPEED;
    }

    turnSpeed = constrain(turnSpeed, -(float)maxSpeed, (float)maxSpeed);

    prevError = yawError;

    writeSpeeds(turnSpeed * V_ROTATE_CW[0], turnSpeed * V_ROTATE_CW[1],
                turnSpeed * V_ROTATE_CW[2], turnSpeed * V_ROTATE_CW[3]);
    updateDeadReckoning();
    delay(10);
  }
  forceStop();
}

void rotateToHeading(float targetHeading, int8_t maxSpeed) {
  robotState = "Rotating to pose";
  updateYaw();
  float startHeading = currentYaw;
  float diff = targetHeading - startHeading;
  while (diff > 180)
    diff -= 360;
  while (diff < -180)
    diff += 360;
  bool clockwise = (diff > 0);
  float degrees = fabs(diff);
  if (degrees < 1.0f) {
    forceStop();
    return;
  }
  rotateDegrees(clockwise, degrees, maxSpeed);
}

// ================================================================
// AUTONOMOUS CAMERA ALIGNMENT
// ================================================================

static bool waitForCameraPose(unsigned long timeout_ms, PoseReply *out) {
  cameraPoseReceived = false;
  sendScanRequest(0);
  unsigned long t0 = millis();
  while (millis() - t0 < timeout_ms) {
    server.handleClient();
    wsServer.loop();
    if (!autonomousMode)
      return false;
    if (cameraPoseReceived) {
      cameraPoseReceived = false;
      PoseReply reply;
      portENTER_CRITICAL(&camMux);
      memcpy(&reply, (const void *)&lastPoseReply, sizeof(reply));
      portEXIT_CRITICAL(&camMux);
      if (reply.status == 1 && reply.pose_valid) {
        if (out)
          *out = reply;
        return true;
      }
      // Partial update (status=0) — keep waiting for DONE
    }
    updateYaw();
    updateDeadReckoning();
    delay(20);
  }
  Serial.println("[AUTO] Camera scan timeout");
  return false;
}

static void alignBurst(const int8_t *vec, int8_t speed, int durationMs) {
  writeSpeeds(vec[0] * speed, vec[1] * speed, vec[2] * speed, vec[3] * speed);
  delay(durationMs);
  forceStop();
}

static bool alignToQR() {
  robotState = "Aligning to QR";
  const float YAW_THRESHOLD_DEG = 6.0f;
  const float APPROACH_DISTANCE_MM = 200.0f;
  const float CONFIDENCE_THRESHOLD = 0.55f;
  const int ALIGN_SPEED = 20;
  const int MAX_ALIGN_STEPS = 12;
  const int MAX_APPROACH_STEPS = 10;

  PoseReply pose;
  if (!waitForCameraPose(12000, &pose))
    return false;

  Serial.printf("[AUTO] Detected: color=%d conf=%.2f yaw=%.1f dist=%.0fmm\n",
                pose.color, pose.confidence, pose.yaw_deg, pose.tz_mm);

  for (int step = 0; step < MAX_ALIGN_STEPS; step++) {
    if (!autonomousMode)
      return false;
    if (fabs(pose.yaw_deg) <= YAW_THRESHOLD_DEG)
      break;
    if (pose.yaw_deg > 0)
      alignBurst(V_STRAFE_R, ALIGN_SPEED, 120);
    else
      alignBurst(V_STRAFE_L, ALIGN_SPEED, 120);
    if (!waitForCameraPose(6000, &pose))
      return false;
  }
  forceStop();
  Serial.printf("[AUTO] Yaw aligned: %.1f°\n", pose.yaw_deg);

  for (int step = 0; step < MAX_APPROACH_STEPS; step++) {
    if (!autonomousMode)
      return false;
    if (pose.tz_mm <= APPROACH_DISTANCE_MM)
      break;
    alignBurst(V_FORWARD, ALIGN_SPEED, 200);
    if (!waitForCameraPose(6000, &pose))
      return false;
  }
  forceStop();
  Serial.printf("[AUTO] Approach complete: dist=%.0fmm conf=%.2f\n", pose.tz_mm,
                pose.confidence);

  if (pose.confidence < CONFIDENCE_THRESHOLD) {
    Serial.println("[AUTO] Low confidence, aborting pickup");
    return false;
  }
  return true;
}

// ================================================================
// WAIT FOR ARM IDLE
// ================================================================

// Blocks until arm finishes current command (busy -> idle transition).
// Returns true if arm went idle, false on timeout or autonomousMode=false.
static bool waitForArmIdle(unsigned long timeout_ms) {
  unsigned long t0 = millis();

  // Phase 1: Wait for arm to go busy (confirms command was received)
  while (millis() - t0 < timeout_ms / 2) {
    if (!autonomousMode)
      return false;
    ArmStatus st;
    memcpy(&st, (const void *)&lastArmStatus, sizeof(st));
    if (st.busy)
      break;
    server.handleClient();
    updateYaw();
    updateDeadReckoning();
    delay(50);
  }

  // Phase 2: Wait for arm to become idle
  while (millis() - t0 < timeout_ms) {
    if (!autonomousMode)
      return false;
    ArmStatus st;
    memcpy(&st, (const void *)&lastArmStatus, sizeof(st));
    if (!st.busy)
      return true;
    server.handleClient();
    updateYaw();
    updateDeadReckoning();
    delay(50);
  }

  Serial.println("[AUTO] Arm idle timeout");
  return false;
}

// ================================================================
// SERVO STEP
// ================================================================

static void servoStep(int idx, int dir) {
  if (idx < 0 || idx > 3)
    return;
  float newAng = servoAngle[idx] + (float)(dir * SERVO_STEP_DEG[idx]);
  newAng = constrain(newAng, 0.0f, 180.0f);
  if (newAng == servoAngle[idx])
    return;
  char cmd[10];
  snprintf(cmd, sizeof(cmd), "SV:%d:%d", idx, (int)round(newAng));
  sendCommandToArm(cmd);
  servoAngle[idx] = newAng;
  Serial.printf("[SERVO] %d -> %.0f deg\n", idx, newAng);
}

// ================================================================
// COLOR NAME HELPER
// ================================================================

const char *colorName(uint8_t c) {
  switch (c) {
  case 1:
    return "RED";
  case 2:
    return "GREEN";
  case 3:
    return "BLUE";
  default:
    return "NONE";
  }
}

// ================================================================
// WEBSOCKET COMMAND HANDLER (text/JSON protocol)
// ================================================================

// Simple JSON string value extractor
String jsonStr(const String &s, const String &key) {
  String k = "\"" + key + "\"";
  int p = s.indexOf(k);
  if (p < 0)
    return "";
  p = s.indexOf(':', p);
  if (p < 0)
    return "";
  p++;
  while (p < (int)s.length() && s[p] == ' ')
    p++;
  if (p >= (int)s.length())
    return "";
  if (s[p] == '"') {
    int e = s.indexOf('"', p + 1);
    return s.substring(p + 1, e);
  }
  int e = p;
  while (e < (int)s.length() && s[e] != ',' && s[e] != '}' && s[e] != ' ')
    e++;
  return s.substring(p, e);
}

void handleCommand(const String &msg) {
  Serial.printf("[CMD] Received: %s\n", msg.c_str());

  String cmd = jsonStr(msg, "cmd");
  String arg = jsonStr(msg, "arg");

  if (cmd == "MOVE") {
    if (!autonomousMode) {
      if (arg == "STOP") {
        forceStop();
        return;
      }
      bool isRotation = (arg == "ROTCW" || arg == "ROTCCW");
      const int8_t *vec = nullptr;
      if (arg == "FWD")
        vec = V_FORWARD;
      else if (arg == "BACK")
        vec = V_BACKWARD;
      else if (arg == "LEFT")
        vec = V_STRAFE_L;
      else if (arg == "RIGHT")
        vec = V_STRAFE_R;
      else if (arg == "ROTCW")
        vec = V_ROTATE_CW;
      else if (arg == "ROTCCW")
        vec = V_ROTATE_CCW;
      else if (arg == "DIAGFR")
        vec = V_DIAG_FR;
      else if (arg == "DIAGFL")
        vec = V_DIAG_FL;
      else if (arg == "DIAGBR")
        vec = V_DIAG_BR;
      else if (arg == "DIAGBL")
        vec = V_DIAG_BL;

      if (vec) {
        moveVec = vec;
        moveSpeed = Motor_speed;
        moveIsRotation = isRotation;
        if (!isRotation) {
          updateYaw();
          moveTargetHeading = currentYaw;
          headingIntegral = 0.0f;
          headingPrevError = 0.0f;
        }
        robotState = stateFromVector(vec);
        moveActive = true;
      }
    }
  } else if (cmd == "ROT143") {
    if (!autonomousMode && !pendingStep) {
      pendingStep = true;
      stepArg = "ROT143";
    }
  } else if (cmd == "POSE") {
    if (!autonomousMode && !pendingStep) {
      pendingStep = true;
      stepArg = "POSE:" + arg;
    }
  } else if (cmd == "SPEED") {
    Motor_speed = (int8_t)constrain(arg.toInt(), 0, 100);
    Serial.printf("Motor Speed Updated: %d\n", Motor_speed);
  } else if (cmd == "ARM") {
    if (arg == "HOME") {
      sendCommandToArm("H");
    } else if (arg == "SCAN_POSE") {
      sendCommandToArm("S");
    } else if (arg == "CTP") {
      sendCommandToArm("CTP");
    } else {
      sendCommandToArm(arg.c_str());
    }
  } else if (cmd == "SSTEP") {
    int joint = arg.charAt(1) - '1';
    int val = arg.substring(3).toInt();
    char armCmd[10];
    snprintf(armCmd, sizeof(armCmd), "SV:%d:%d", joint, val);
    sendCommandToArm(armCmd);
  } else if (cmd == "AUTO") {
    if (arg == "CALIBRATE") {
      autonomousMode = true;
      autoTrigger = 1;
      Serial.println("[AUTO] CALIBRATION MODE");
    } else if (arg == "COMPETE") {
      autonomousMode = true;
      autoTrigger = 2;
      Serial.println("[AUTO] COMPETITION MODE");
    } else if (arg == "TOGGLE") {
      autonomousMode = !autonomousMode;
      if (autonomousMode) {
        autoTrigger = 1;
        Serial.println("AUTONOMOUS MODE (default: calibrate)");
      } else {
        Serial.println("MANUAL MODE");
      }
    } else {
      autonomousMode = (arg == "ON");
      if (autonomousMode) {
        autoTrigger = 1;
      } else {
        Serial.println("MANUAL MODE");
      }
    }
    forceStop();
  } else if (cmd == "SCAN") {
    if (arg == "QR") {
      newQrResult = false;
      robotState = "Scanning QR";
      sendScanRequest(0);
      Serial.println("[SCAN] QR scan requested");
    } else if (arg == "PLAT") {
      newQrResult = false;
      robotState = "Scanning Platform";
      sendScanRequest(1);
      Serial.println("[SCAN] Platform scan requested");
    } else if (arg == "STOP") {
      scanInProgress = false;
      newQrResult = false;
      Serial.println("[SCAN] Scan stopped by user");
    }
  } else if (cmd == "SERVO") {
    int idx = arg.substring(0, arg.indexOf(':')).toInt();
    String dirStr = arg.substring(arg.indexOf(':') + 1);
    int dir = (dirStr == "UP" || dirStr == "1") ? 1 : -1;
    servoStep(idx, dir);
  }
}

// ================================================================
// DEAD RECKONING — update global position from robot-frame movement
// ================================================================
// Vectors: {FL, FR, RL, RR} with mecanum convention.
// Positive X = forward, Positive Y = left in robot frame.
void trackMovement(const int8_t vector[], float dist_mm) {
  // Determine robot-frame displacement from mecanum vector
  float dx_r = 0, dy_r = 0;
  if (vector[0] == 1 && vector[1] == -1 && vector[2] == -1 && vector[3] == 1) {
    dx_r = dist_mm; // FORWARD
  } else if (vector[0] == -1 && vector[1] == 1 && vector[2] == 1 &&
             vector[3] == -1) {
    dx_r = -dist_mm; // BACKWARD
  } else if (vector[0] == -1 && vector[1] == -1 && vector[2] == -1 &&
             vector[3] == -1) {
    dy_r = dist_mm; // STRAFE LEFT
  } else if (vector[0] == 1 && vector[1] == 1 && vector[2] == 1 &&
             vector[3] == 1) {
    dy_r = -dist_mm; // STRAFE RIGHT
  } else if (vector[0] == -1 && vector[1] == 1 && vector[2] == -1 &&
             vector[3] == 1) {
    dx_r = dist_mm;
    dy_r = dist_mm; // DIAG FL
  } else if (vector[0] == 1 && vector[1] == 1 && vector[2] == -1 &&
             vector[3] == -1) {
    dx_r = dist_mm;
    dy_r = -dist_mm; // DIAG FR
  } else if (vector[0] == -1 && vector[1] == -1 && vector[2] == 1 &&
             vector[3] == 1) {
    dx_r = -dist_mm;
    dy_r = dist_mm; // DIAG BL
  } else if (vector[0] == 1 && vector[1] == -1 && vector[2] == 1 &&
             vector[3] == -1) {
    dx_r = -dist_mm;
    dy_r = -dist_mm; // DIAG BR
  } else {
    return; // unknown vector
  }

  float heading_rad = currentYaw * PI / 180.0f;
  float c = cosf(heading_rad), s = sinf(heading_rad);
  posX += dx_r * c - dy_r * s;
  posY += dx_r * s + dy_r * c;
}

// ================================================================
// CONTINUOUS ENCODER DEAD RECKONING — called from loop()
// Mecanum kinematics: 1980 ticks/rev, 80mm wheels
// Motor order: FL(0), FR(1), BL(2), BR(3)
// V_FORWARD = {1,-1,-1,1} → encoder sign convention
// ================================================================
void updateDeadReckoning() {
  unsigned long now = millis();
  float dt = (now - lastEncoderMs) / 1000.0f;
  if (dt < 0.02f)
    return;        // max 50Hz
  if (dt > 0.5f) { // first call or long gap — just initialize
    lastEncoderMs = now;
    int32_t enc[4];
    if (readEncoders(enc)) {
      for (int i = 0; i < 4; i++)
        lastEncoders[i] = enc[i];
      encodersInitialized = true;
    }
    return;
  }
  lastEncoderMs = now;

  int32_t enc[4];
  if (!readEncoders(enc))
    return;
  if (!encodersInitialized) {
    for (int i = 0; i < 4; i++)
      lastEncoders[i] = enc[i];
    encodersInitialized = true;
    return;
  }

  // Deltas in encoder ticks
  float d[4];
  for (int i = 0; i < 4; i++) {
    d[i] = (float)(enc[i] - lastEncoders[i]);
    lastEncoders[i] = enc[i];
  }

  // Mecanum inverse kinematics (robot frame)
  // V_FORWARD = {1,-1,-1,1} convention
  float vx_r = (d[0] - d[1] - d[2] + d[3]) / 4.0f * MM_PER_TICK;
  float vy_r = -(d[0] + d[1] + d[2] + d[3]) / 4.0f * MM_PER_TICK;

  // Raw velocity from encoders (mm/s)
  float rawVx = vx_r / dt;
  float rawVy = vy_r / dt;

  // EMA low-pass filter
  velX = EMA_ALPHA * rawVx + (1.0f - EMA_ALPHA) * velX;
  velY = EMA_ALPHA * rawVy + (1.0f - EMA_ALPHA) * velY;
  float rawSpeed = sqrtf(rawVx * rawVx + rawVy * rawVy);
  currentSpeed = EMA_ALPHA * rawSpeed + (1.0f - EMA_ALPHA) * currentSpeed;
  speedKmh = currentSpeed * 0.0036f; // mm/s -> km/h

  // Rotate to global frame using current heading (updateYaw called from loop())
  float heading_rad = currentYaw * PI / 180.0f;
  float c = cosf(heading_rad), s = sinf(heading_rad);
  posX += vx_r * c - vy_r * s;
  posY += vx_r * s + vy_r * c;
}

// ================================================================
// CONTROLLER HTML (embedded)
// ================================================================

const char CONTROLLER_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
    <title>Glitch — Robot Controller</title>
    <meta name="theme-color" content="#0a0e17">
    <meta name="apple-mobile-web-app-capable" content="yes">
    <style>
        :root {
            --bg: #0a0e17;
            --panel: #111827;
            --panel-2: #1f2937;
            --border: rgba(99,102,241,0.25);
            --text: #f1f5f9;
            --muted: #94a3b8;
            --accent: #818cf8;
            --accent-glow: rgba(129,140,248,0.4);
            --ok: #34d399;
            --warn: #fbbf24;
            --err: #fb7185;
        }
        * { box-sizing: border-box; margin: 0; padding: 0; -webkit-tap-highlight-color: transparent; }
        html, body { background: var(--bg); color: var(--text);
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', system-ui, sans-serif;
            min-height: 100vh; }
        body { padding: 14px; padding-bottom: env(safe-area-inset-bottom); max-width: 520px; margin: 0 auto; }
        h1 { font-size: 22px; font-weight: 800; margin-bottom: 4px;
            background: linear-gradient(135deg, #818cf8, #22d3ee);
            -webkit-background-clip: text; background-clip: text; color: transparent; }
        .sub { color: var(--muted); font-size: 12px; margin-bottom: 14px; display:flex; align-items:center; gap:8px; }
        .card { background: var(--panel); border: 1px solid var(--border);
            border-radius: 14px; padding: 14px; margin-bottom: 12px;
            box-shadow: 0 4px 16px rgba(0,0,0,0.25); }
        .card h2 { font-size: 11px; color: var(--muted); font-weight: 600;
            text-transform: uppercase; letter-spacing: 1px; margin-bottom: 12px; }
        .dot { width: 10px; height: 10px; border-radius: 50%; background: var(--err);
            transition: box-shadow 200ms; flex-shrink:0; }
        .dot.ok { background: var(--ok); box-shadow: 0 0 10px var(--ok); }
        .dot.warn { background: var(--warn); box-shadow: 0 0 10px var(--warn); }
        button { background: var(--panel-2); color: var(--text); border: 1px solid var(--border);
            border-radius: 10px; padding: 14px 8px; font-size: 14px; font-weight: 600;
            cursor: pointer; user-select: none; -webkit-user-select: none;
            transition: background 80ms, transform 80ms, box-shadow 200ms; touch-action: manipulation; }
        button:active { background: var(--accent); transform: scale(0.96);
            box-shadow: 0 0 16px var(--accent-glow); }
        button.active { background: var(--accent); border-color: var(--accent);
            box-shadow: 0 0 12px var(--accent-glow); }
        button.danger:active { background: var(--err); }
        .pad { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; aspect-ratio: 1.4; }
        .pad button { font-size: 26px; padding: 0; }
        .grid-2 { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
        .grid-3 { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 8px; }
        .slider-row { display: flex; align-items: center; gap: 10px; }
        .slider-row input[type="range"] { flex: 1; accent-color: var(--accent); height: 6px; }
        .slider-row .val { font-family: 'Courier New', monospace; font-weight: 700;
            min-width: 40px; text-align: right; font-size: 16px; }
        .telemetry { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
        .tile { background: var(--panel-2); border-radius: 10px; padding: 10px; }
        .tile .label { font-size: 10px; color: var(--muted); text-transform: uppercase; letter-spacing: 0.5px; }
        .tile .val { font-family: 'Courier New', monospace; font-size: 20px; font-weight: 700; margin-top: 2px; }
        .tile .val.small { font-size: 14px; }
        .log { font-family: 'Courier New', monospace; font-size: 11px;
            color: var(--muted); max-height: 130px; overflow-y: auto;
            background: var(--panel-2); border-radius: 8px; padding: 8px; }
        .log .err { color: var(--err); }
        .log .ok  { color: var(--ok); }
        .section-label { font-size: 10px; color: var(--muted); margin: 4px 0 6px; }
        .step-btn { padding: 6px 14px !important; font-size: 16px !important; font-weight: 700 !important; }
        @keyframes pulse { 0%,100%{opacity:1} 50%{opacity:0.5} }
        button.scanning { animation: pulse 1.5s ease-in-out infinite; }
        @media (orientation: landscape) and (max-height: 500px) {
            body { padding: 8px; max-width: none; }
            #topPanels { display: flex; gap: 8px; }
            #topPanels > .card { flex: 1; margin-bottom: 0; }
            #topPanels .log { max-height: 80px; }
            #controlPanels { display: flex; flex-wrap: wrap; gap: 8px; }
            #controlPanels > .card { flex: 1 1 45%; margin-bottom: 0; min-width: 200px; }
            .pad { aspect-ratio: 1.2; }
            .pad button { font-size: 22px; }
            button { padding: 10px 6px; font-size: 13px; }
        }
        .btn-r{background:#dc2626;border-color:#ef4444;color:#fff}
        .btn-r:active{background:#ef4444;box-shadow:0 0 12px rgba(239,68,68,0.5)}
        .btn-g{background:#16a34a;border-color:#22c55e;color:#fff}
        .btn-g:active{background:#22c55e;box-shadow:0 0 12px rgba(34,197,94,0.5)}
        .btn-b{background:#2563eb;border-color:#3b82f6;color:#fff}
        .btn-b:active{background:#3b82f6;box-shadow:0 0 12px rgba(59,130,246,0.5)}
        .btn-auto{background:#f59e0b;border-color:#f59e0b;color:#000;font-size:16px;font-weight:700;width:100%;margin-bottom:8px}
        .btn-auto:active{background:#d97706;box-shadow:0 0 12px rgba(245,158,11,0.5)}
        .btn-auto.active{background:#ef4444;border-color:#ef4444;color:#fff}
        .btn-cal{background:#22d3ee;border-color:#22d3ee;color:#000;font-size:14px;font-weight:700;width:100%;margin-bottom:4px}
        .btn-cal:active{background:#06b6d4;box-shadow:0 0 12px rgba(34,211,238,0.5)}
        .btn-cal.active{background:#ef4444;border-color:#ef4444;color:#fff}
    </style>
</head>
<body>
    <h1>Glitch</h1>
    <div class="sub" id="connStatus"><span class="dot"></span><span>connecting…</span></div>

    <div id="controlPanels">
    <div class="card">
        <h2>Drive</h2>
        <div class="pad">
            <button id="btnDiagFL" data-cmd="MOVE" data-arg="DIAGFL">&#8598;</button>
            <button id="btnFwd"    data-cmd="MOVE" data-arg="FWD">&#9650;</button>
            <button id="btnDiagFR" data-cmd="MOVE" data-arg="DIAGFR">&#8599;</button>
            <button id="btnLeft"   data-cmd="MOVE" data-arg="LEFT">&#9664;</button>
            <button id="btnStop"   data-cmd="MOVE" data-arg="STOP" class="danger">&#9632;</button>
            <button id="btnRight"  data-cmd="MOVE" data-arg="RIGHT">&#9654;</button>
            <button id="btnDiagBL" data-cmd="MOVE" data-arg="DIAGBL">&#8601;</button>
            <button id="btnBack"   data-cmd="MOVE" data-arg="BACK">&#9660;</button>
            <button id="btnDiagBR" data-cmd="MOVE" data-arg="DIAGBR">&#8600;</button>
        </div>
        <div class="grid-2" style="margin-top:8px;">
            <button id="btnRotCCW" data-cmd="MOVE" data-arg="ROTCCW">Rotate &#8634;</button>
            <button id="btnRotCW"  data-cmd="MOVE" data-arg="ROTCW">Rotate &#8635;</button>
        </div>
        <div style="margin-top:8px;">
            <button id="btnRot143"  data-cmd="ROT143" class="btn-auto">143&deg; &#8635;</button>
        </div>
        <div class="section-label" style="margin-top:14px">Motor Speed</div>
        <div class="slider-row">
            <input type="range" id="speed" min="0" max="100" value="25">
            <span class="val" id="speedVal">25</span>
        </div>
        <div class="section-label" style="margin-top:14px">Pose Heading</div>
        <div class="grid-3">
            <button data-cmd="POSE" data-arg="0">0&deg;</button>
            <button data-cmd="POSE" data-arg="90">90&deg;</button>
            <button data-cmd="POSE" data-arg="180">180&deg;</button>
        </div>
    </div>

    <div class="card">
        <h2>Arm</h2>
        <div class="grid-3" style="margin-bottom:10px;">
            <button data-cmd="ARM" data-arg="HOME" class="danger">Home</button>
            <button data-cmd="ARM" data-arg="CTP">Car &rarr; Plt</button>
        </div>
    </div>

    <div class="card">
        <h2>Step Servo</h2>
        <div style="display:grid;grid-template-columns:1fr auto auto;gap:6px 10px;align-items:center;font-size:13px;">
            <span>S1 Base</span>     <button class="step-btn" data-joint="0" data-dir="-">-</button> <button class="step-btn" data-joint="0" data-dir="+">+</button>
            <span>S2 Shoulder</span> <button class="step-btn" data-joint="1" data-dir="-">-</button> <button class="step-btn" data-joint="1" data-dir="+">+</button>
            <span>S3 Elbow</span>    <button class="step-btn" data-joint="2" data-dir="-">-</button> <button class="step-btn" data-joint="2" data-dir="+">+</button>
            <span>S4 Wrist</span>    <button class="step-btn" data-joint="3" data-dir="-">-</button> <button class="step-btn" data-joint="3" data-dir="+">+</button>
            <span>S5 Claw</span>     <button class="step-btn" data-joint="4" data-dir="-">-</button> <button class="step-btn" data-joint="4" data-dir="+">+</button>
        </div>
        <div style="margin-top:8px;display:flex;align-items:center;gap:8px;">
            <label style="font-size:10px;color:var(--muted);text-transform:uppercase;">Step °</label>
            <input type="number" id="stepSize" value="5" min="1" max="90" style="width:50px;background:var(--panel-2);color:var(--text);border:1px solid var(--border);border-radius:6px;padding:4px 6px;font-size:13px;text-align:center;">
        </div>
    </div>

    <div class="card">
        <h2>Mode</h2>
        <button id="btnCalibrate" class="btn-cal" data-cmd="AUTO" data-arg="CALIBRATE">CALIBRATE</button>
        <button id="btnCompete" class="btn-auto" data-cmd="AUTO" data-arg="COMPETE">COMPETITION</button>
        <button id="btnScanQR" data-cmd="SCAN" data-arg="QR" style="width:100%">Scan QR</button>
    </div>
    </div>

    <div id="topPanels">
    <div class="card" id="camCard">
        <h2>Camera</h2>
        <div class="telemetry">
            <div class="tile" style="grid-column:span 2"><div class="label">Heading</div><div class="val" id="heading">0.0&deg;</div></div>
            <div class="tile"><div class="label">QR Msg</div><div class="val small" id="camMsg">--</div></div>
            <div class="tile"><div class="label">Color</div><div class="val" id="camColor">--</div></div>
            <div class="tile"><div class="label">Confidence</div><div class="val" id="camConf">--</div></div>
            <div class="tile"><div class="label">Pose (x,y,z)</div><div class="val small" id="camPose">--</div></div>
        </div>
    </div>

    <div class="card">
        <h2>Event Log</h2>
        <div class="log" id="log"></div>
    </div>
    </div>

    <script>
        var $=function(id){return document.getElementById(id)};
        var log=function(msg,cls){
            var el=$('log');var t=new Date().toLocaleTimeString();
            el.innerHTML='<div class="'+cls+'">['+t+'] '+msg+'</div>'+el.innerHTML;
            while(el.children.length>80)el.removeChild(el.lastChild);
        };
        var setStatus=function(cls,text){
            $('connStatus').innerHTML='<span class="dot '+cls+'"></span><span>'+text+'</span>';
        };
        var armBusy=false;
        function setArmBusy(busy){
            armBusy=busy;
            log(busy?'Arm: busy':'Arm: idle',busy?'err':'ok');
        }
        var lastPose={valid:false,color:0,tx:0,ty:0,tz:0,yaw:0,conf:0,msg:''};
        var scanActive=false;
        var scanMiss=0;

        function send(data){
            fetch('/cmd',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)})
            .catch(function(){});
        }

        function pollStatus(){
            fetch('/status').then(function(r){return r.json()}).then(function(s){
                setStatus('ok','connected to base');
                setArmBusy(s.arm_busy);

                if(s.autonomous){
                    $('btnCalibrate').classList.add('active');
                    $('btnCompete').classList.add('active');
                    $('btnCalibrate').textContent='RUNNING...';
                    $('btnCompete').textContent='RUNNING...';
                }else{
                    $('btnCalibrate').classList.remove('active');
                    $('btnCompete').classList.remove('active');
                    $('btnCalibrate').textContent='CALIBRATE';
                    $('btnCompete').textContent='COMPETITION';
                }

                if(s.current_yaw!=null)$('heading').textContent=parseFloat(s.current_yaw).toFixed(1)+'\u00B0';

                var scanBtn=$('btnScanQR');
                if(s.scan_in_progress){
                    scanActive=true;
                    scanBtn.textContent='Scanning...';
                    scanBtn.classList.add('scanning');
                }else{
                    scanActive=false;
                    scanBtn.textContent='Scan QR';
                    scanBtn.classList.remove('scanning');
                }

                if(s.color&&s.color!=='NONE'){
                    $('camCard').style.display='';
                    $('camColor').textContent=s.color;
                    $('camConf').textContent=s.confidence!=null?(parseFloat(s.confidence)).toFixed(2):'--';
                    $('camPose').textContent=(s.tx_mm!=null?s.tx_mm:'--')+', '+(s.ty_mm!=null?s.ty_mm:'--')+', '+(s.distance_mm!=null?s.distance_mm:'--');
                }
                if(s.qr_result){
                    var qr=s.qr_result;
                    lastPose.valid=qr.pose_valid;
                    lastPose.color=qr.color;
                    lastPose.tx=qr.tx_mm;
                    lastPose.ty=qr.ty_mm;
                    lastPose.tz=qr.tz_mm;
                    lastPose.yaw=qr.yaw_deg;
                    lastPose.conf=qr.confidence;
                    lastPose.msg=qr.qr_msg||'';
                    $('camMsg').textContent=qr.qr_msg||'(no text)';
                    $('camColor').textContent=qr.color_name||'--';
                    $('camConf').textContent=qr.confidence!=null?(parseFloat(qr.confidence)).toFixed(2):'--';
                    $('camPose').textContent=(qr.tx_mm!=null?parseFloat(qr.tx_mm).toFixed(0):'--')+', '+(qr.ty_mm!=null?parseFloat(qr.ty_mm).toFixed(0):'--')+', '+(qr.tz_mm!=null?parseFloat(qr.tz_mm).toFixed(0):'--');
                    if(qr.pose_valid)log('QR decoded: '+(qr.qr_msg||'(no text)')+' color='+(qr.color_name||'?'),'ok');
                    scanMiss=0;
                }else if(!s.qr_pending){
                }else{
                    scanMiss++;
                    if(scanMiss>1)$('camMsg').textContent='No QR found (#'+scanMiss+')';
                }
            }).catch(function(){
                setStatus('','disconnected');
            });
        }
        setInterval(pollStatus,500);
        pollStatus();

        var driveBtns=document.querySelectorAll('.pad button[data-cmd="MOVE"], #btnRotCCW, #btnRotCW');
        var sendDriveCmd=function(e){
            e.preventDefault();
            driveBtns.forEach(function(btn){btn.classList.remove('active')});
            this.classList.add('active');
            send({cmd:'MOVE',arg:this.dataset.arg});
        };
        driveBtns.forEach(function(b){
            b.addEventListener('touchstart',sendDriveCmd,{passive:false});
            b.addEventListener('mousedown',sendDriveCmd);
        });

        var rot143Btn=document.getElementById('btnRot143');
        if(rot143Btn){
            var sendRot143=function(e){
                e.preventDefault();
                send({cmd:'ROT143',arg:''});
            };
            rot143Btn.addEventListener('touchstart',sendRot143,{passive:false});
            rot143Btn.addEventListener('mousedown',sendRot143);
        }

        document.querySelectorAll('button[data-cmd]').forEach(function(b){
            if(b.closest('.pad')||b.id==='btnScanQR')return;
            b.addEventListener('click',function(){send({cmd:b.dataset.cmd,arg:b.dataset.arg})});
        });
        // btnCalibrate/btnCompete handled by generic data-cmd handler + pollStatus()
        $('btnScanQR').addEventListener('click',function(){
            if(scanActive){
                send({cmd:'SCAN',arg:'STOP'});
                scanActive=false;
            }else{
                send({cmd:'SCAN',arg:'QR'});
                scanActive=true;
                $('camMsg').textContent='--';
                $('camColor').textContent='--';
                $('camConf').textContent='--';
                $('camPose').textContent='--';
                lastPose={valid:false,color:0,tx:0,ty:0,tz:0,yaw:0,conf:0,msg:''};
            }
        });

        var speedT;
        $('speed').addEventListener('input',function(e){$('speedVal').textContent=e.target.value});
        $('speed').addEventListener('input',function(e){
            clearTimeout(speedT);
            speedT=setTimeout(function(){send({cmd:'SPEED',arg:parseInt(e.target.value)})},120);
        });

        var stepJoints=['J1','J2','J3','J4','J5'];
        var stepInterval=null;
        function startStep(btn){
            if(btn.disabled)return;
            var j=parseInt(btn.dataset.joint);
            var dir=btn.dataset.dir==='+'?1:-1;
            var deg=parseInt($('stepSize').value)||5;
            send({cmd:'SSTEP',arg:stepJoints[j]+','+(dir*deg)});
            stopStep();
            stepInterval=setInterval(function(){
                send({cmd:'SSTEP',arg:stepJoints[j]+','+(dir*deg)});
            },150);
        }
        function stopStep(){
            if(stepInterval){clearInterval(stepInterval);stepInterval=null;}
        }
        document.querySelectorAll('.step-btn').forEach(function(btn){
            btn.addEventListener('pointerdown',function(e){e.preventDefault();startStep(btn);});
            btn.addEventListener('pointerup',stopStep);
            btn.addEventListener('pointerleave',stopStep);
            btn.addEventListener('pointercancel',stopStep);
        });

        var keyMap={ArrowUp:'FWD',ArrowDown:'BACK',ArrowLeft:'LEFT',ArrowRight:'RIGHT'};
        var keysDown=new Set();
        document.addEventListener('keydown',function(e){
            if(keyMap[e.key]&&!keysDown.has(e.key)){
                keysDown.add(e.key);send({cmd:'MOVE',arg:keyMap[e.key]});
            }
            if(e.key===' '){e.preventDefault();send({cmd:'MOVE',arg:'STOP'});}
        });
        document.addEventListener('keyup',function(e){
            if(keyMap[e.key]){keysDown.delete(e.key);send({cmd:'MOVE',arg:'STOP'});}
        });
    </script>
</body>
</html>
)rawliteral";

// ================================================================
// DASHBOARD HTML (embedded) — display only, no controls
// ================================================================

const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Robot Dashboard</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#0a0a0f;color:#e0e0e0;font-family:'Segoe UI',system-ui,sans-serif;height:100vh;overflow:hidden}
.grid{display:grid;grid-template-columns:1fr 320px;grid-template-rows:auto 1fr;gap:8px;padding:8px;height:100vh}
.panel{background:#12121a;border:1px solid #2a2a3a;border-radius:8px;padding:12px;overflow:hidden}
.panel h3{font-size:11px;text-transform:uppercase;letter-spacing:1.5px;color:#666;margin-bottom:8px}
.cam-wrap{position:relative;width:100%;height:100%;display:flex;align-items:center;justify-content:center;background:#000;border-radius:6px;overflow:hidden}
.cam-wrap img{max-width:100%;max-height:100%;object-fit:contain}
.cam-label{position:absolute;top:8px;left:8px;background:rgba(0,0,0,0.7);color:#0f0;font-size:10px;padding:2px 8px;border-radius:4px;font-family:monospace}
.right-stack{display:flex;flex-direction:column;gap:8px}
.map-wrap{flex:1;position:relative;min-height:180px}
.map-wrap canvas{width:100%;height:100%;border-radius:6px}
.info-row{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.info-row3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px}
.stat{text-align:center}
.stat .val{font-size:28px;font-weight:700;font-family:'Courier New',monospace;color:#0af;line-height:1.2}
.stat .val.sm{font-size:16px}
.stat .lbl{font-size:10px;color:#666;text-transform:uppercase;letter-spacing:1px}
.compass-wrap{position:relative;width:100%;aspect-ratio:1;max-height:200px;margin:0 auto}
.compass-wrap canvas{width:100%;height:100%}
.conn-dot{display:inline-block;width:7px;height:7px;border-radius:50%;margin-right:5px;vertical-align:middle}
.conn-dot.on{background:#0f0;box-shadow:0 0 6px #0f0}
.conn-dot.off{background:#f00;box-shadow:0 0 6px #f00}
.conn-dot.warn{background:#fa0;box-shadow:0 0 6px #fa0}
.hdr{display:flex;align-items:center;justify-content:space-between;margin-bottom:8px}
.hdr .title{font-size:13px;font-weight:600;letter-spacing:0.5px}
.mode-badge{display:inline-block;padding:2px 8px;border-radius:4px;font-size:10px;font-weight:700;font-family:monospace;letter-spacing:1px}
.mode-auto{background:rgba(251,191,36,0.2);color:#fbbf24;border:1px solid rgba(251,191,36,0.4)}
.mode-manual{background:rgba(52,211,153,0.2);color:#34d399;border:1px solid rgba(52,211,153,0.4)}
.state-badge{display:inline-block;padding:2px 8px;border-radius:4px;font-size:10px;font-weight:700;font-family:monospace;letter-spacing:1px;margin-left:6px}
.state-idle{background:rgba(148,163,184,0.2);color:#94a3b8;border:1px solid rgba(148,163,184,0.4)}
.state-moving{background:rgba(56,189,248,0.2);color:#38bdf8;border:1px solid rgba(56,189,248,0.4)}
.state-rotating{background:rgba(251,146,60,0.2);color:#fb923c;border:1px solid rgba(251,146,60,0.4)}
.state-scanning{background:rgba(168,85,247,0.2);color:#a855f7;border:1px solid rgba(168,85,247,0.4)}
.state-arm{background:rgba(236,72,153,0.2);color:#ec4899;border:1px solid rgba(236,72,153,0.4)}
.qr-grid{display:grid;grid-template-columns:1fr 1fr;gap:6px}
.qr-tile{background:#1a1a2a;border-radius:6px;padding:8px}
.qr-tile .lbl{font-size:9px;color:#666;text-transform:uppercase;letter-spacing:0.5px}
.qr-tile .vl{font-family:'Courier New',monospace;font-size:14px;font-weight:700;color:#0af;margin-top:1px}
.qr-tile .vl.sm{font-size:12px}
.qr-msg{grid-column:span 2;background:#1a1a2a;border-radius:6px;padding:8px;font-family:'Courier New',monospace;font-size:12px;color:#a855f7;text-align:center;min-height:32px;display:flex;align-items:center;justify-content:center}
</style>
</head>
<body>
<div class="grid">
  <div class="panel" style="grid-row:1/3">
    <div class="hdr">
      <h3>Camera Feed</h3>
      <span><span class="conn-dot off" id="camDot"></span><span id="camStatus" style="font-size:10px;color:#666">Connecting...</span></span>
    </div>
    <div class="cam-wrap">
      <img id="camStream" src="" alt="Camera stream">
      <div class="cam-label" id="camFps"></div>
    </div>
  </div>
  <div class="right-stack" style="grid-row:1/3">
    <div class="panel">
      <div class="hdr">
        <h3>Status</h3>
        <span><span class="mode-badge mode-manual" id="modeBadge">MANUAL</span><span class="state-badge state-idle" id="stateBadge">Idle</span></span>
      </div>
      <div class="info-row3">
        <div class="stat"><div class="val sm" id="posX">0.0</div><div class="lbl">X (mm)</div></div>
        <div class="stat"><div class="val sm" id="posY">0.0</div><div class="lbl">Y (mm)</div></div>
        <div class="stat"><div class="val sm" id="headingVal">0.0&deg;</div><div class="lbl">Heading</div></div>
      </div>
      <div class="info-row3" style="margin-top:8px">
        <div class="stat"><div class="val sm" id="velX">0</div><div class="lbl">Vx (km/h)</div></div>
        <div class="stat"><div class="val sm" id="velY">0</div><div class="lbl">Vy (km/h)</div></div>
        <div class="stat"><div class="val sm" id="speed">0</div><div class="lbl">Speed (km/h)</div></div>
      </div>
    </div>
    <div class="panel">
      <h3>QR Code Results</h3>
      <div class="qr-grid">
        <div class="qr-tile"><div class="lbl">Color</div><div class="vl" id="qrColor">--</div></div>
        <div class="qr-tile"><div class="lbl">Confidence</div><div class="vl" id="qrConf">--</div></div>
        <div class="qr-tile"><div class="lbl">Pose X (mm)</div><div class="vl sm" id="qrTx">--</div></div>
        <div class="qr-tile"><div class="lbl">Pose Y (mm)</div><div class="vl sm" id="qrTy">--</div></div>
        <div class="qr-tile"><div class="lbl">Distance (mm)</div><div class="vl sm" id="qrTz">--</div></div>
        <div class="qr-tile"><div class="lbl">Yaw (deg)</div><div class="vl sm" id="qrYaw">--</div></div>
        <div class="qr-msg" id="qrMsg">No QR scanned</div>
      </div>
    </div>
    <div class="panel">
      <h3>Heading</h3>
      <div class="compass-wrap"><canvas id="compassCvs"></canvas></div>
    </div>
    <div class="panel map-wrap">
      <h3>Global Map</h3>
      <canvas id="mapCvs"></canvas>
    </div>
  </div>
</div>
<script>
const CAM_IP='192.168.4.100';
const CAM_PORT=81;
let posX=0,posY=0,heading=0,velX=0,velY=0,speed=0,autoMode=false;
let trail=[];
let camRetryCount=0;
let camLastFrame=0;
let camTimer=null;
const camImg=document.getElementById('camStream');
const camDot=document.getElementById('camDot');
const camStatus=document.getElementById('camStatus');
const mapCvs=document.getElementById('mapCvs');
const compassCvs=document.getElementById('compassCvs');
const mapCtx=mapCvs.getContext('2d');
const compCtx=compassCvs.getContext('2d');

camImg.onload=function(){
  camDot.className='conn-dot on';
  camStatus.textContent='Connected';
  camLastFrame=Date.now();
  camRetryCount=0;
};
camImg.onerror=function(){
  camDot.className='conn-dot off';
  camStatus.textContent='Disconnected';
  camRetryCount++;
  var delay=Math.min(camRetryCount*1500,8000);
  setTimeout(function(){
    camImg.src='http://'+CAM_IP+':'+CAM_PORT+'/stream?t='+Date.now();
  },delay);
};

camTimer=setInterval(function(){
  var stale=Date.now()-camLastFrame;
  if(stale>5000&&camLastFrame>0){
    camDot.className='conn-dot warn';
    camStatus.textContent='Stalled \u2014 reconnecting...';
    camImg.src='http://'+CAM_IP+':'+CAM_PORT+'/stream?t='+Date.now();
    camLastFrame=Date.now();
  }
},3000);

camImg.src='http://'+CAM_IP+':'+CAM_PORT+'/stream?t='+Date.now();

function pollStatus(){
  fetch('/api/status').then(function(r){return r.json()}).then(function(d){
    posX=d.x; posY=d.y; heading=d.heading||0;
    velX=d.velX||0; velY=d.velY||0; speed=d.speed||0;
    autoMode=d.autonomous||false;
    document.getElementById('posX').textContent=posX.toFixed(0);
    document.getElementById('posY').textContent=posY.toFixed(0);
    document.getElementById('headingVal').textContent=heading.toFixed(1)+'\u00B0';
    document.getElementById('velX').textContent=velX.toFixed(0);
    document.getElementById('velY').textContent=velY.toFixed(0);
    document.getElementById('speed').textContent=speed.toFixed(0);
    var mb=document.getElementById('modeBadge');
    if(autoMode){mb.textContent='AUTONOMOUS';mb.className='mode-badge mode-auto';}
    else{mb.textContent='MANUAL';mb.className='mode-badge mode-manual';}

    var st=d.robot_state||'Idle';
    var sb=document.getElementById('stateBadge');
    sb.textContent=st;
    sb.className='state-badge';
    if(st==='Idle')sb.classList.add('state-idle');
    else if(st.indexOf('Rotating')>=0)sb.classList.add('state-rotating');
    else if(st.indexOf('Scanning')>=0||st.indexOf('Aligning')>=0)sb.classList.add('state-scanning');
    else if(st.indexOf('Arm')>=0)sb.classList.add('state-arm');
    else sb.classList.add('state-moving');

    if(d.qr_result){
      var qr=d.qr_result;
      document.getElementById('qrColor').textContent=qr.color_name||'--';
      document.getElementById('qrConf').textContent=qr.confidence!=null?parseFloat(qr.confidence).toFixed(2):'--';
      document.getElementById('qrTx').textContent=qr.tx_mm!=null?parseFloat(qr.tx_mm).toFixed(0):'--';
      document.getElementById('qrTy').textContent=qr.ty_mm!=null?parseFloat(qr.ty_mm).toFixed(0):'--';
      document.getElementById('qrTz').textContent=qr.tz_mm!=null?parseFloat(qr.tz_mm).toFixed(0):'--';
      document.getElementById('qrYaw').textContent=qr.yaw_deg!=null?parseFloat(qr.yaw_deg).toFixed(1):'--';
      document.getElementById('qrMsg').textContent=qr.qr_msg||'No text decoded';
      if(qr.color_name&&qr.color_name!=='NONE'){
        var cMap={'RED':'#ef4444','GREEN':'#22c55e','BLUE':'#3b82f6'};
        document.getElementById('qrColor').style.color=cMap[qr.color_name]||'#0af';
      }
    }

    trail.push({x:posX,y:posY});
    if(trail.length>1000)trail.shift();
    drawMap();
    drawCompass();
  }).catch(function(){});
}
setInterval(pollStatus,200);
pollStatus();

function resizeCanvas(cvs){
  const r=cvs.parentElement.getBoundingClientRect();
  const dpr=window.devicePixelRatio||1;
  cvs.width=r.width*dpr; cvs.height=r.height*dpr;
  cvs.style.width=r.width+'px'; cvs.style.height=r.height+'px';
  return dpr;
}

function drawMap(){
  const dpr=resizeCanvas(mapCvs);
  const ctx=mapCtx;
  const W=mapCvs.width, H=mapCvs.height;
  ctx.fillStyle='#0a0a12'; ctx.fillRect(0,0,W,H);
  var maxDist=300;
  for(var i=0;i<trail.length;i++){
    var ax=Math.abs(trail[i].x), ay=Math.abs(trail[i].y);
    if(ax>maxDist)maxDist=ax;
    if(ay>maxDist)maxDist=ay;
  }
  var ax=Math.abs(posX), ay=Math.abs(posY);
  if(ax>maxDist)maxDist=ax;
  if(ay>maxDist)maxDist=ay;
  maxDist=Math.max(maxDist*1.4,150);
  var scale=Math.min(W,H)/(maxDist*2);
  var ox=W/2, oy=H/2;
  ctx.strokeStyle='#1a1a2a'; ctx.lineWidth=1;
  var gridStep=100;
  if(maxDist>1000)gridStep=500;
  else if(maxDist>500)gridStep=200;
  for(var g=-2000;g<=2000;g+=gridStep){
    var x=ox+g*scale, y=oy+g*scale;
    if(x>0&&x<W){ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,H);ctx.stroke();}
    if(y>0&&y<H){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(W,y);ctx.stroke();}
  }
  if(trail.length>1){
    ctx.strokeStyle='rgba(0,170,255,0.4)'; ctx.lineWidth=2*dpr;
    ctx.beginPath();
    ctx.moveTo(ox-trail[0].y*scale, oy-trail[0].x*scale);
    for(var i=1;i<trail.length;i++) ctx.lineTo(ox-trail[i].y*scale, oy-trail[i].x*scale);
    ctx.stroke();
  }
  var rx=ox-posY*scale, ry=oy-posX*scale;
  ctx.fillStyle='#0af';
  ctx.beginPath(); ctx.arc(rx,ry,6*dpr,0,Math.PI*2); ctx.fill();
  ctx.fillStyle='rgba(0,170,255,0.2)';
  ctx.beginPath(); ctx.arc(rx,ry,14*dpr,0,Math.PI*2); ctx.fill();
  var hrad=heading*Math.PI/180;
  ctx.strokeStyle='#f55'; ctx.lineWidth=2*dpr;
  ctx.beginPath(); ctx.moveTo(rx,ry);
  ctx.lineTo(rx-Math.sin(hrad)*25*dpr, ry-Math.cos(hrad)*25*dpr);
  ctx.stroke();
  ctx.fillStyle='#555'; ctx.font='bold '+(10*dpr)+'px monospace';
  ctx.textAlign='center';
  ctx.fillText('E',ox, oy-H/2+14*dpr);
  ctx.fillText('W',ox, oy+H/2-6*dpr);
  ctx.fillText('N',ox-W/2+8*dpr, oy+4*dpr);
  ctx.fillText('S',ox+W/2-8*dpr, oy+4*dpr);
  ctx.textAlign='left';
  ctx.fillStyle='#444'; ctx.font=(9*dpr)+'px monospace';
  ctx.fillText('Scale: '+maxDist.toFixed(0)+'mm',8*dpr,H-8*dpr);
}

function drawCompass(){
  const dpr=resizeCanvas(compassCvs);
  const ctx=compCtx;
  const W=compassCvs.width, H=compassCvs.height;
  const cx=W/2, cy=H/2, R=Math.min(W,H)/2-10*dpr;
  ctx.clearRect(0,0,W,H);
  ctx.strokeStyle='#2a2a3a'; ctx.lineWidth=2*dpr;
  ctx.beginPath(); ctx.arc(cx,cy,R,0,Math.PI*2); ctx.stroke();
  const labels=[['N',0],['E',90],['S',180],['W',270]];
  labels.forEach(function(l){
    const a=(l[1]-90)*Math.PI/180;
    ctx.fillStyle=(l[0]==='N')?'#f55':'#666';
    ctx.font='bold '+(11*dpr)+'px sans-serif';
    ctx.textAlign='center'; ctx.textBaseline='middle';
    ctx.fillText(l[0], cx+Math.cos(a)*(R-14*dpr), cy+Math.sin(a)*(R-14*dpr));
  });
  const ha=(heading-90)*Math.PI/180;
  ctx.save(); ctx.translate(cx,cy); ctx.rotate(ha);
  ctx.fillStyle='#0af';
  ctx.beginPath(); ctx.moveTo(R-8*dpr,0);
  ctx.lineTo(-8*dpr,-6*dpr); ctx.lineTo(-8*dpr,6*dpr);
  ctx.closePath(); ctx.fill();
  ctx.fillStyle='#333';
  ctx.beginPath(); ctx.moveTo(-R+8*dpr,0);
  ctx.lineTo(8*dpr,-4*dpr); ctx.lineTo(8*dpr,4*dpr);
  ctx.closePath(); ctx.fill();
  ctx.restore();
}

window.addEventListener('resize',function(){drawMap();drawCompass();});
drawMap(); drawCompass();
</script>
</body>
</html>
)rawliteral";

// ================================================================
// SETUP
// ================================================================

void setup() {
  Serial.begin(115200);

  // I2C: 100kHz — safe for both MPU6050 and Hiwonder motor driver
  Wire.begin(SDA_PIN, SCL_PIN, 100000);
  Wire.setTimeOut(20);
  delay(500);

  // MPU6050 DMP init
  mpu.initialize();
  if (mpu.dmpInitialize() == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    dmpReady = true;
    Serial.println("MPU6050 DMP Ready");
  } else {
    Serial.println("MPU6050 DMP INIT FAILED");
  }

  // Motor driver init (type=3, polarity=0 = normal 4-wheel mecanum)
  uint8_t motorType = 3;
  writeBytes(REG_MOTOR_TYPE, &motorType, 1);
  delay(10);
  uint8_t polarity = 0;
  writeBytes(REG_MOTOR_PHASE, &polarity, 1);
  delay(10);
  uint8_t resetData[16] = {0};
  writeBytes(REG_ENCODER_TOTAL, resetData, 16);
  forceStop();
  Serial.println("Motor System Ready");

  // ================= WIFI AP =================
  WiFi.softAP(AP_SSID, AP_PASS, WIFI_CHANNEL);
  delay(100);
  esp_wifi_set_ps(WIFI_PS_NONE);

  Serial.println("\n========================================");
  Serial.printf("  WiFi AP: %s\n", AP_SSID);
  Serial.printf("  IP: %s\n", WiFi.softAPIP().toString().c_str());
  Serial.printf("  BASE MAC: %s\n", WiFi.softAPmacAddress().c_str());
  Serial.printf("  Channel: %d\n", WiFi.channel());
  Serial.println("========================================");

  // ================= ESP NOW =================
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW INIT FAILED");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Add Camera peer (ESP-NOW stays for camera only)
  esp_now_peer_info_t camPeer = {};
  memcpy(camPeer.peer_addr, cameraAddress, 6);
  camPeer.channel = WIFI_CHANNEL;
  camPeer.encrypt = false;
  camPeer.ifidx = WIFI_IF_AP;

  if (esp_now_add_peer(&camPeer) != ESP_OK) {
    Serial.println("Failed to Add Camera Peer");
  } else {
    Serial.println("Camera ESP-NOW Peer Added");
  }

  Serial.println("ESP-NOW READY (camera only)");

  // ================= WEB SERVER =================

  // Serve controller page at root
  server.on("/", HTTP_GET,
            []() { server.send(200, "text/html", CONTROLLER_HTML); });

  // Status endpoint — polled by GUI every 500ms
  server.on("/status", HTTP_GET, []() {
    String json = "{";
    json += "\"arm_busy\":" + String(lastArmStatus.busy ? "true" : "false");
    json += ",\"autonomous\":" + String(autonomousMode ? "true" : "false");
    json += ",\"qr_pending\":" + String(newQrResult ? "false" : "true");
    json +=
        ",\"scan_in_progress\":" + String(scanInProgress ? "true" : "false");
    json += ",\"scan_status\":\"" +
            String(scanInProgress ? "Accumulating" : "DONE") + "\"";
    json += ",\"current_yaw\":" + String(currentYaw, 1);

    if (newQrResult) {
      json += ",\"qr_result\":{";
      json += "\"pose_valid\":" + String(lastQrValid);
      json += ",\"color\":" + String(lastQrColor);
      json += ",\"color_name\":\"" + String(colorName(lastQrColor)) + "\"";
      json += ",\"confidence\":" + String(lastQrConf, 2);
      json += ",\"tx_mm\":" + String(lastQrTx, 0);
      json += ",\"ty_mm\":" + String(lastQrTy, 0);
      json += ",\"tz_mm\":" + String(lastQrTz, 0);
      json += ",\"yaw_deg\":" + String(lastQrYaw, 1);
      json += ",\"qr_msg\":\"QR Color " + String(colorName(lastQrColor)) + "\"";
      json += ",\"scan_status\":\"" +
              String(lastScanStatus == 1 ? "DONE" : "Accumulating") + "\"";
      json += "}";
    }
    json += "}";
    server.send(200, "application/json", json);
  });

  // Command endpoint — GUI POSTs commands here
  server.on("/cmd", HTTP_POST, []() {
    String body = server.arg("plain");
    if (body.length() > 0) {
      handleCommand(body);
    }
    server.send(200, "application/json", "{\"ok\":true}");
  });

  // Dashboard page — display-only monitoring
  server.on("/dashboard", HTTP_GET,
            []() { server.send(200, "text/html", DASHBOARD_HTML); });

  // Dashboard API — returns position, heading, velocity, mode as JSON
  server.on("/api/status", HTTP_GET, []() {
    String json = "{";
    json += "\"x\":" + String(posX, 1);
    json += ",\"y\":" + String(posY, 1);
    json += ",\"heading\":" + String(currentYaw, 1);
    json += ",\"velX\":" + String(velX * 0.0036f, 2);
    json += ",\"velY\":" + String(velY * 0.0036f, 2);
    json += ",\"speed\":" + String(speedKmh, 2);
    json += ",\"autonomous\":" + String(autonomousMode ? "true" : "false");
    json += ",\"mode\":\"" + String(autonomousMode ? "Autonomous" : "Manual") +
            "\"";
    json += ",\"robot_state\":\"" + robotState + "\"";
    json +=
        ",\"scan_in_progress\":" + String(scanInProgress ? "true" : "false");
    json += ",\"arm_busy\":" + String(lastArmStatus.busy ? "true" : "false");

    if (newQrResult) {
      json += ",\"qr_result\":{";
      json += "\"pose_valid\":" + String(lastQrValid);
      json += ",\"color\":" + String(lastQrColor);
      json += ",\"color_name\":\"" + String(colorName(lastQrColor)) + "\"";
      json += ",\"confidence\":" + String(lastQrConf, 2);
      json += ",\"tx_mm\":" + String(lastQrTx, 0);
      json += ",\"ty_mm\":" + String(lastQrTy, 0);
      json += ",\"tz_mm\":" + String(lastQrTz, 0);
      json += ",\"yaw_deg\":" + String(lastQrYaw, 1);
      json += ",\"qr_msg\":\"QR Color " + String(colorName(lastQrColor)) + "\"";
      json += ",\"scan_status\":\"" +
              String(lastScanStatus == 1 ? "DONE" : "Accumulating") + "\"";
      json += "}";
    }
    json += "}";
    server.send(200, "application/json", json);
  });

  server.begin();
  Serial.println("Web server started on port 80");

  wsServer.begin();
  wsServer.onEvent(onWsEvent);
  Serial.printf("WebSocket server started on port %d\n", WS_PORT);

  Serial.printf("Dashboard: http://%s/dashboard\n",
                WiFi.softAPIP().toString().c_str());
}

// ================================================================
// LOOP
// ================================================================

void loop() {
  server.handleClient();
  wsServer.loop();

  // =================== I2C SEQUENCING ===================
  // updateYaw() reads MPU6050 (0x68), updateMotors()/updateDeadReckoning()
  // read/write motor driver (0x34). Sequence: yaw first, then motor/encoder,
  // so the I2C bus is free for the next yaw read.
  if (!autonomousMode) {
    updateYaw();
    if (moveActive) {
      delayMicroseconds(200);
      updateMotors();
    }
    updateDeadReckoning();
  } else {
    updateYaw();
    updateDeadReckoning();
  }

  // =================== SCAN TIMEOUT WATCHDOG ===================
  if (scanInProgress && millis() - scanStartMs > 40000) {
    scanInProgress = false;
    Serial.println("[CAM] Scan timeout — forcing scanInProgress=false");
  }

  // =================== DEFERRED ESP-NOW DATA ===================
  if (cameraPoseReceived) {
    cameraPoseReceived = false;
    PoseReply reply;
    portENTER_CRITICAL(&camMux);
    memcpy(&reply, (const void *)&lastPoseReply, sizeof(reply));
    portEXIT_CRITICAL(&camMux);
    Serial.printf(
        "[CAM] Pose: valid=%d color=%d conf=%.2f yaw=%.1f tz=%.1f est=%d\n",
        reply.pose_valid, reply.color, reply.confidence, reply.yaw_deg,
        reply.tz_mm, reply.estimated);
    if (reply.status == 1 || (reply.pose_valid && reply.confidence > 0.0f)) {
      lastQrValid = reply.pose_valid;
      lastQrColor = reply.color;
      lastQrTx = reply.tx_mm;
      lastQrTy = reply.ty_mm;
      lastQrTz = reply.tz_mm;
      lastQrYaw = reply.yaw_deg;
      lastQrConf = reply.confidence;
      newQrResult = true;
    }
    // Camera still accumulating — reset watchdog so base doesn't timeout
    // mid-retry
    if (reply.status == 0) {
      scanStartMs = millis();
    }
  }
  if (armStatusReceived) {
    armStatusReceived = false;
    ArmStatus st;
    memcpy(&st, (const void *)&lastArmStatus, sizeof(st));
    Serial.printf("[ARM] Status: busy=%d\n", st.busy);
  }

  // =================== STEP HANDLER (POSE only) ===================
  if (pendingStep && !moveActive) {
    if (stepArg.startsWith("POSE:")) {
      float heading = stepArg.substring(5).toFloat();
      for (int attempt = 0; attempt < 3; attempt++) {
        rotateToHeading(heading, Motor_speed);
        updateYaw();
        float err = heading - currentYaw;
        if (err > 180)
          err -= 360;
        if (err < -180)
          err += 360;
        if (fabs(err) < 1.0f)
          break;
      }
    } else if (stepArg == "ROT143") {
      rotateDegrees(true, 143, Motor_speed);
    }
    pendingStep = false;
  }

  // =================== ESP-NOW DEBUG ===================
  static unsigned long lastStatusUpdate = 0;
  if (millis() - lastStatusUpdate > 5000) {
    lastStatusUpdate = millis();
    Serial.printf("[ESP-NOW RX] total=%lu bytes=%lu pose=%lu\n",
                  (unsigned long)espNowRecvCount,
                  (unsigned long)espNowRecvBytes,
                  (unsigned long)espNowRecvMatchPose);
  }

  // =================== AUTONOMOUS SEQUENCES ===================
  // autoTrigger 1 = Calibration (test moves for recording)
  // autoTrigger 2 = Competition (full task sequence)
  // Toggle autonomous OFF to stop mid-sequence.

  // --- CALIBRATION SEQUENCE ---
  if (autonomousMode && autoTrigger == 1) {
    autoTrigger = 0;
    Serial.println("[AUTO] === Starting calibration sequence ===");

    Serial.println("[AUTO] Step 1: Forward 1m (record encoder data)");
    moveDistanceKp(V_FORWARD, Motor_speed, 1.0, TICKS_FWD_BWD);
    if (!autonomousMode)
      goto calEnd;

    delay(2500);

    Serial.println("[AUTO] Strafe right for 1 meter (calibration)");
    moveDistanceKp(V_STRAFE_R, Motor_speed, 1.0, TICKS_STRAFE);
    if (!autonomousMode)
      goto calEnd;

    delay(2500);

    // rotate 180 degrees
    rotateDegrees(false, 180.0, Motor_speed);
    if (!autonomousMode)
      goto calEnd;

    delay(2500);

    Serial.println("[AUTO] Diagonal FWD 1m (record encoder data)");
    moveDistanceKp(V_DIAG_FR, Motor_speed, 1.0, TICKS_DIAG);
    if (!autonomousMode)
      goto calEnd;

    Serial.println("[AUTO] === Calibration sequence complete ===");
  calEnd:
    forceStop();
    autonomousMode = false;
    Serial.println("[AUTO] Autonomous sequence ended");
  }

  // --- COMPETITION SEQUENCE ---
  if (autonomousMode && autoTrigger == 2) {
    autoTrigger = 0;
    Serial.println("[AUTO] === Starting competition sequence ===");

    // Step 1: FWD to red box
    Serial.println("[AUTO] Step 1: Forward to red box");
    moveDistanceKp(V_FORWARD, Motor_speed, DIST_1, TICKS_FWD_BWD);
    if (!autonomousMode)
      goto autoEnd;

    // Step 2: Rotate 90 CCW
    Serial.println("[AUTO] Step 2: Rotate 90 CCW");
    rotateDegrees(false, 90, Motor_speed);
    if (!autonomousMode)
      goto autoEnd;

    /* Step 3: Pose correction to 0
    Serial.println("[AUTO] Step 3: Pose correction to 0");
    rotateToHeading(0, Motor_speed);
    if (!autonomousMode)
      goto autoEnd;*/

    /*sep 3.5: Pose correction to 0 again
    Serial.println("[AUTO] Step 4: Rotate 0 degrees");
    rotateToHeading(0, Motor_speed);
    if (!autonomousMode)
      goto autoEnd;*/

    // Step 4 FWD to red box area
    Serial.println("[AUTO] Step 5: Forward to red box area");
    moveDistanceKp(V_FORWARD, Motor_speed, DIST_2, TICKS_FWD_BWD);
    if (!autonomousMode)
      goto autoEnd;

    // Step 5: Red to Floor
    Serial.println("[AUTO] Step 5: Red -> Floor");
    sendCommandToArm("RTF");
    waitForArmIdle(15000);
    if (!autonomousMode)
      goto autoEnd;

    // Step 6: BWD centering
    Serial.println("[AUTO] Step 6: Backward centering");
    moveDistanceKp(V_BACKWARD, Motor_speed, DIST_3, TICKS_FWD_BWD);
    if (!autonomousMode)
      goto autoEnd;

    // Step 7: Pose correction to 0
    Serial.println("[AUTO] Step 7: Pose correction to 0");
    rotateToHeading(0, Motor_speed);
    if (!autonomousMode)
      goto autoEnd;

    // Step 7.5: Pose correction to 0 again
    Serial.println("[AUTO] Step 7.5: Rotate 0 degrees");
    rotateToHeading(0, Motor_speed);
    if (!autonomousMode)
      goto autoEnd;

    // Step 8: RIGHT to green box
    Serial.println("[AUTO] Step 8: Right to green box");
    moveDistanceKp(V_STRAFE_R, Motor_speed, DIST_4 / 2, TICKS_STRAFE);
    if (!autonomousMode)
      goto autoEnd;

    rotateToHeading(0, Motor_speed);
    if (!autonomousMode)
      goto autoEnd;

    rotateToHeading(0, Motor_speed);
    if (!autonomousMode)
      goto autoEnd;

    moveDistanceKp(V_STRAFE_R, Motor_speed, DIST_4 / 2, TICKS_STRAFE);
    if (!autonomousMode)
      goto autoEnd;

    // Step 9: Green to Floor
    Serial.println("[AUTO] Step 9: Green -> Floor");
    sendCommandToArm("GTF");
    waitForArmIdle(15000);
    if (!autonomousMode)
      goto autoEnd;

    // Step 10: LEFT centering
    Serial.println("[AUTO] Step 10: Left centering");
    moveDistanceKp(V_STRAFE_L, Motor_speed, DIST_5, TICKS_STRAFE);
    if (!autonomousMode)
      goto autoEnd;

    // Step 11: Rotate 180
    Serial.println("[AUTO] Step 11: Rotate 180");
    rotateDegrees(true, 180, Motor_speed);
    if (!autonomousMode)
      goto autoEnd;
    /*
        // Step 12: Pose correction to 180
        Serial.println("[AUTO] Step 12: Pose correction to 180");
        rotateToHeading(180, Motor_speed);
        if (!autonomousMode)
          goto autoEnd;
    */
    // Step 12.5: Pose correction to 180 again
    Serial.println("[AUTO] Step 12.5: Rotate 180 degrees");
    rotateToHeading(180, Motor_speed);
    if (!autonomousMode)
      goto autoEnd;

    // Step 13: FWD
    Serial.println("[AUTO] Step 13: Forward");
    moveDistanceKp(V_FORWARD, Motor_speed, DIST_6, TICKS_FWD_BWD);
    if (!autonomousMode)
      goto autoEnd;

    // Step 14: DIAG to blue box
    Serial.println("[AUTO] Step 14: Diagonal to blue box");
    moveDistanceKp(V_DIAG_FR, Motor_speed, DIST_7, TICKS_DIAG);
    if (!autonomousMode)
      goto autoEnd;

    // Step 15: Blue to Floor
    Serial.println("[AUTO] Step 15: Blue -> Floor");
    sendCommandToArm("BTF");
    waitForArmIdle(20000);

    Serial.println("[AUTO] === Competition sequence complete ===");
  autoEnd:
    forceStop();
    autonomousMode = false;
    Serial.println("[AUTO] Autonomous sequence ended");
  }
}
