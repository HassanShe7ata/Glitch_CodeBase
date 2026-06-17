// =========================== BASE ESP32 (WiFi AP + HTTP + ESP-NOW) ===========================
// Phone → WiFi "GLITCH" → http://192.168.4.1 → HTTP POST → ESP-NOW → Arm/Camera

#include <WiFi.h>
#include <WebServer.h>
#include <esp_wifi.h>
#include <math.h>
#include <Wire.h>
#include <esp_now.h>
#include "MPU6050_6Axis_MotionApps20.h"

// ================= WIFI AP =================
const char* AP_SSID = "GLITCH";
const char* AP_PASS = "Gl1tch2024!Secure";
const uint8_t WIFI_CHANNEL = 11;

// ================= WEB SERVER =================
WebServer server(80);

// ================= ESP-NOW SHARED ENUMS =================
enum ArmColorCode : uint8_t {
  ARM_COLOR_UNKNOWN = 0,
  ARM_COLOR_R = 1,
  ARM_COLOR_G = 2,
  ARM_COLOR_B = 3,
};

// ================= ESP NOW =================

// Arm ESP32 MAC Address
uint8_t armAddress[] = {0x68, 0xFE, 0x71, 0x12, 0x5D, 0xA8};

typedef struct struct_message {
  char command[10];
} struct_message;

struct_message armMessage;

// =================== CAMERA ESP-NOW ===================
static uint8_t cameraAddress[] = {0x94, 0xA9, 0x90, 0x08, 0xB2, 0xB8}; // must match camera's WiFi STA MAC

// ESP-NOW packet types (must match camera firmware)
enum EspNowPacketType : uint8_t {
  ESPNOW_TYPE_SCAN_REQ   = 0x20,
  ESPNOW_TYPE_POSE_REPLY = 0x30,
};

// ESP-NOW packet: Base → Camera (scan request)
struct __attribute__((packed)) ScanRequest {
  uint8_t type;        // ESPNOW_TYPE_SCAN_REQ (0x20)
  uint8_t task_id;
  uint8_t mode;        // 0=scan_qr, 1=scan_platform
  uint8_t reserved;
};

// ESP-NOW packet: Camera → Base (pose reply)
struct __attribute__((packed)) PoseReply {
  uint8_t type;        // ESPNOW_TYPE_POSE_REPLY (0x30)
  uint8_t task_id;
  uint8_t pose_valid;
  uint8_t color;       // 0=unknown, 1=R, 2=G, 3=B
  uint8_t estimated;
  float tx_mm;
  float ty_mm;
  float tz_mm;
  float yaw_deg;
  float confidence;
  uint8_t status;      // 0=Accumulating, 1=DONE
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
  uint8_t type;   // 0 = status
  uint8_t busy;   // 1 = busy, 0 = idle
  uint8_t pad[2];
};

// Latest camera pose data (updated by ESP-NOW callback)
static volatile bool cameraPoseReceived = false;
static volatile PoseReply lastPoseReply;

// Latest arm status (updated by ESP-NOW callback)
static volatile bool armStatusReceived = false;
static volatile ArmStatus lastArmStatus;

// ESP-NOW recv debug counter
static volatile uint32_t espNowRecvCount = 0;
static volatile uint32_t espNowRecvBytes = 0;
static volatile uint32_t espNowRecvMatchPose = 0;
static volatile uint32_t espNowRecvMatchArm = 0;
static volatile uint32_t espNowRecvNoMatch = 0;

// Scan progress tracking
static volatile bool scanInProgress = false;
static volatile uint8_t lastScanStatus = 0;  // 0=Accumulating, 1=DONE
static unsigned long scanStartMs = 0;
static portMUX_TYPE camMux = portMUX_INITIALIZER_UNLOCKED;

// Last QR scan result (persists until new scan)
static bool lastQrValid = false;
static uint8_t lastQrColor = 0;
static float lastQrTx = 0, lastQrTy = 0, lastQrTz = 0, lastQrYaw = 0, lastQrConf = 0;
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

// --- I2C Registers ---
#define I2C_ADDR 0x34
#define REG_MOTOR_TYPE 0x14
#define REG_MOTOR_PHASE 0x15
#define REG_FIXED_SPEED 0x33
#define REG_ENCODER_TOTAL 0x3C

// --- Pins ---
#define SDA_PIN 21
#define SCL_PIN 22

bool autonomousMode = false;
bool autoTrigger = 0;

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

// --- CALIBRATED TICK CONSTANTS (Blynk values × 97/80, distance in meters) ---
const float TICKS_FWD_BWD = 6717.25f;
const float TICKS_STRAFE  = 7581.71f;
const float TICKS_DIAG    = 9548.44f;
const float TICKS_ROTATE  = 8730.00f;

int8_t Motor_speed = 25;

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
  Serial.print("ESP-NOW Send Status: ");
  if (status == ESP_NOW_SEND_SUCCESS)
    Serial.println("SUCCESS");
  else
    Serial.println("FAILED");
}

// ESP-NOW receive callback — JUST STORE DATA, no heavy operations
// Runs in WiFi task — MUST return immediately.
void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (!mac || !data || len < 1) return;
  bool fromArm = (memcmp(mac, armAddress, 6) == 0);
  bool fromCam = (memcmp(mac, cameraAddress, 6) == 0);
  if (!fromArm && !fromCam) return;

  espNowRecvCount++;
  espNowRecvBytes += len;
  if (fromCam && len == (int)sizeof(PoseReply) && data[0] == ESPNOW_TYPE_POSE_REPLY) {
    portENTER_CRITICAL(&camMux);
    memcpy((void *)&lastPoseReply, data, sizeof(PoseReply));
    portEXIT_CRITICAL(&camMux);
    cameraPoseReceived = true;
    lastScanStatus = lastPoseReply.status;
    if (lastPoseReply.status == 1) scanInProgress = false;
    espNowRecvMatchPose++;
  } else if (fromArm && len == (int)sizeof(ArmStatus) && data[0] == 0) {
    memcpy((void *)&lastArmStatus, data, sizeof(ArmStatus));
    armStatusReceived = true;
    espNowRecvMatchArm++;
  } else {
    espNowRecvNoMatch++;
  }
}

void sendCommandToArm(const char *cmd) {
  strncpy(armMessage.command, cmd, sizeof(armMessage.command) - 1);
  armMessage.command[sizeof(armMessage.command) - 1] = '\0';
  esp_err_t result =
      esp_now_send(armAddress, (uint8_t *)&armMessage, sizeof(armMessage));
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
  req.type = ESPNOW_TYPE_SCAN_REQ;
  req.task_id = ++taskCounter;
  req.mode = mode;
  esp_err_t result = esp_now_send(cameraAddress, (uint8_t *)&req, sizeof(req));
  if (result == ESP_OK) {
    scanInProgress = true;
    lastScanStatus = 0;
    scanStartMs = millis();
    Serial.printf("[CAM] Scan request sent: task=%d mode=%d\n", req.task_id, req.mode);
  } else {
    scanInProgress = false;
    Serial.printf("[CAM] Scan request FAILED: task=%d mode=%d err=%d\n", req.task_id, req.mode, result);
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

  esp_err_t result = esp_now_send(armAddress, (uint8_t *)&data, sizeof(data));
  Serial.printf("[BASE] Camera pose forwarded to arm: valid=%d color=%d "
                "tx=%.0f ty=%.0f tz=%.0f yaw=%.1f conf=%.2f (%s)\n",
                data.pose_valid, data.color, data.tx_mm, data.ty_mm, data.tz_mm,
                data.yaw_deg, data.confidence,
                result == ESP_OK ? "OK" : "FAILED");
}

// ================================================================
// I2C HELPERS
// ================================================================

bool writeBytes(uint8_t reg, uint8_t *data, size_t len) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(reg);
  for (size_t i = 0; i < len; i++)
    Wire.write(data[i]);
  return (Wire.endTransmission() == 0);
}

bool readEncoders(int32_t *data) {
  Wire.setClock(40000);
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(REG_ENCODER_TOTAL);
  if (Wire.endTransmission() != 0)
    return false;
  if (Wire.requestFrom((uint8_t)I2C_ADDR, (uint8_t)16) != 16)
    return false;
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

bool writeSpeeds(int8_t v1, int8_t v2, int8_t v3, int8_t v4) {
  Wire.setClock(40000);
  int8_t speeds[4] = {v1, v2, v3, v4};
  return writeBytes(REG_FIXED_SPEED, (uint8_t *)speeds, 4);
}

void updateMotors() {
  if (!moveActive) return;
  updateYaw();
  if (moveIsRotation) {
    writeSpeeds(constrain((int)moveSpeed * moveVec[0], -100, 100),
                constrain((int)moveSpeed * moveVec[1], -100, 100),
                constrain((int)moveSpeed * moveVec[2], -100, 100),
                constrain((int)moveSpeed * moveVec[3], -100, 100));
  } else {
    float yawError = moveTargetHeading - currentYaw;
    if (yawError > 180) yawError -= 360;
    if (yawError < -180) yawError += 360;
    int8_t gyroCorr = (int8_t)constrain(yawError * KP_GYRO, -127.0f, 127.0f);
    writeSpeeds(constrain((int)(moveSpeed * moveVec[0]) + gyroCorr, -100, 100),
                constrain((int)(moveSpeed * moveVec[1]) - gyroCorr, -100, 100),
                constrain((int)(moveSpeed * moveVec[2]) + gyroCorr, -100, 100),
                constrain((int)(moveSpeed * moveVec[3]) - gyroCorr, -100, 100));
  }
}

void forceStop() {
  moveActive = false;
  for (int i = 0; i < 5; i++) {
    if (writeSpeeds(0, 0, 0, 0)) {
      break;
    }
    delay(5);
  }
}

// ================================================================
// MECANUM WHEEL MIXING (for binary protocol compatibility)
// ================================================================

void computeMecanumSpeeds(int8_t throttle, int8_t steering, int8_t rotation, int8_t speeds[4]) {
  int16_t fl =  (int16_t)throttle - (int16_t)steering + (int16_t)rotation;
  int16_t fr = -(int16_t)throttle - (int16_t)steering + (int16_t)rotation;
  int16_t bl = -(int16_t)throttle + (int16_t)steering + (int16_t)rotation;
  int16_t br =  (int16_t)throttle + (int16_t)steering + (int16_t)rotation;
  speeds[0] = (int8_t)constrain(fl, -100, 100);
  speeds[1] = (int8_t)constrain(fr, -100, 100);
  speeds[2] = (int8_t)constrain(bl, -100, 100);
  speeds[3] = (int8_t)constrain(br, -100, 100);
}

// ================================================================
// MPU6050 DMP
// ================================================================

void updateYaw() {
  if (dmpReady) {
    Wire.setClock(400000);
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
  int32_t startEncoders[4], currentEncoders[4];
  long targetTicks = lroundf(fabs(distance * tickConstant));
  int8_t localMinTorque = MIN_TORQUE;
  maxSpeed = max(maxSpeed, localMinTorque);

  updateYaw();
  targetHeading = currentYaw;

  if (!readEncoders(startEncoders))
    return;
  long error = targetTicks;
  uint8_t loopCounter = 0;
  uint8_t i2cErrors = 0;

  while (true) {
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
          Serial.println("[ERR] I2C encoder read failed 5 times, aborting move");
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
    if (yawError > 180) yawError -= 360;
    if (yawError < -180) yawError += 360;
    int8_t gyroCorr = (int8_t)constrain(yawError * KP_GYRO, -127.0f, 127.0f);

    writeSpeeds(constrain((int)(baseSpeed * vector[0]) + gyroCorr, -100, 100),
                constrain((int)(baseSpeed * vector[1]) - gyroCorr, -100, 100),
                constrain((int)(baseSpeed * vector[2]) + gyroCorr, -100, 100),
                constrain((int)(baseSpeed * vector[3]) - gyroCorr, -100, 100));

    if (loopCounter > MOVE_TIMEOUT_ITERATIONS) {
      Serial.println("[ERR] Move timed out (blocked or stalled), aborting");
      break;
    }
    delay(15);
  }
  forceStop();
}

void rotateDegrees(bool clockwise, float degrees, int8_t maxSpeed) {
  updateYaw();
  float startHeading = currentYaw;
  float target = clockwise ? (startHeading + degrees) : (startHeading - degrees);

  while (true) {
    updateYaw();
    float yawError = target - currentYaw;
    if (yawError > 180) yawError -= 360;
    if (yawError < -180) yawError += 360;

    if (fabs(yawError) < 1.0f) break;

    int8_t turnSpeed = (fabs(yawError) < 20.0f) ? 20 : maxSpeed;
    if (yawError < 0) turnSpeed = -turnSpeed;

    writeSpeeds(turnSpeed * V_ROTATE_CW[0], turnSpeed * V_ROTATE_CW[1],
                turnSpeed * V_ROTATE_CW[2], turnSpeed * V_ROTATE_CW[3]);
    delay(10);
  }
  forceStop();
}

// ================================================================
// AUTONOMOUS CAMERA ALIGNMENT
// ================================================================

static bool waitForCameraPose(unsigned long timeout_ms, PoseReply *out) {
  cameraPoseReceived = false;
  sendScanRequest(0);
  unsigned long t0 = millis();
  while (millis() - t0 < timeout_ms) {
    if (!autonomousMode)
      return false;
    if (cameraPoseReceived) {
      cameraPoseReceived = false;
      PoseReply reply;
      portENTER_CRITICAL(&camMux);
      memcpy(&reply, (const void *)&lastPoseReply, sizeof(reply));
      portEXIT_CRITICAL(&camMux);
      if (reply.status == 1 && reply.pose_valid) {
        if (out) *out = reply;
        return true;
      }
      // Partial update (status=0) — keep waiting for DONE
    }
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
                pose.color, pose.confidence,
                pose.yaw_deg, pose.tz_mm);

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
  Serial.printf("[AUTO] Approach complete: dist=%.0fmm conf=%.2f\n",
                pose.tz_mm, pose.confidence);

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
  delay(300);  // Give arm time to receive command and go busy
  unsigned long t0 = millis();
  while (millis() - t0 < timeout_ms) {
    if (!autonomousMode) return false;
    ArmStatus st;
    memcpy(&st, (const void *)&lastArmStatus, sizeof(st));
    if (!st.busy) return true;
    server.handleClient();
    delay(100);
  }
  Serial.println("[AUTO] Arm idle timeout");
  return false;
}

// ================================================================
// SERVO STEP
// ================================================================

static void servoStep(int idx, int dir) {
  if (idx < 0 || idx > 3) return;
  float newAng = servoAngle[idx] + (float)(dir * SERVO_STEP_DEG[idx]);
  newAng = constrain(newAng, 0.0f, 180.0f);
  if (newAng == servoAngle[idx]) return;
  char cmd[10];
  snprintf(cmd, sizeof(cmd), "SV:%d:%d", idx, (int)round(newAng));
  sendCommandToArm(cmd);
  servoAngle[idx] = newAng;
  Serial.printf("[SERVO] %d -> %.0f deg\n", idx, newAng);
}

// ================================================================
// COLOR NAME HELPER
// ================================================================

const char* colorName(uint8_t c) {
  switch (c) {
    case 1: return "RED";
    case 2: return "GREEN";
    case 3: return "BLUE";
    default: return "NONE";
  }
}

// ================================================================
// WEBSOCKET COMMAND HANDLER (text/JSON protocol)
// ================================================================

// Simple JSON string value extractor
String jsonStr(const String &s, const String &key) {
  String k = "\"" + key + "\"";
  int p = s.indexOf(k);
  if (p < 0) return "";
  p = s.indexOf(':', p);
  if (p < 0) return "";
  p++;
  while (p < (int)s.length() && s[p] == ' ') p++;
  if (p >= (int)s.length()) return "";
  if (s[p] == '"') {
    int e = s.indexOf('"', p + 1);
    return s.substring(p + 1, e);
  }
  int e = p;
  while (e < (int)s.length() && s[e] != ',' && s[e] != '}' && s[e] != ' ') e++;
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
      if (arg == "FWD")        vec = V_FORWARD;
      else if (arg == "BACK")  vec = V_BACKWARD;
      else if (arg == "LEFT")  vec = V_STRAFE_L;
      else if (arg == "RIGHT") vec = V_STRAFE_R;
      else if (arg == "ROTCW") vec = V_ROTATE_CW;
      else if (arg == "ROTCCW") vec = V_ROTATE_CCW;
      else if (arg == "DIAGFR") vec = V_DIAG_FR;
      else if (arg == "DIAGFL") vec = V_DIAG_FL;
      else if (arg == "DIAGBR") vec = V_DIAG_BR;
      else if (arg == "DIAGBL") vec = V_DIAG_BL;

      if (vec) {
        moveVec = vec;
        moveSpeed = Motor_speed;
        moveIsRotation = isRotation;
        if (!isRotation) {
          updateYaw();
          moveTargetHeading = currentYaw;
        }
        moveActive = true;
      }
    }
  } else if (cmd == "STEP") {
    if (!autonomousMode && !pendingStep) {
      pendingStep = true;
      stepArg = arg;
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
      } else if (arg == "CAM_PICKUP") {
        if (lastQrValid) {
          sendCameraPoseToArm();
          newQrResult = false;
          Serial.println("[CAM] Pose forwarded to arm for IK pickup");
        } else {
          Serial.println("[CAM] No valid pose to send");
        }
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
    if (arg == "TOGGLE") {
      autonomousMode = !autonomousMode;
    } else {
      autonomousMode = (arg == "ON");
    }
    if (autonomousMode) {
      autoTrigger = 1;
      Serial.println("AUTONOMOUS MODE");
    } else {
      Serial.println("MANUAL MODE");
    }
    forceStop();
    } else if (cmd == "SCAN") {
      if (arg == "QR") {
        newQrResult = false;
        sendScanRequest(0);
        Serial.println("[SCAN] QR scan requested");
      } else if (arg == "PLAT") {
        newQrResult = false;
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
// WEBSOCKET EVENT HANDLER
// ================================================================

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
        <div class="section-label" style="margin-top:14px">Motor Speed</div>
        <div class="slider-row">
            <input type="range" id="speed" min="0" max="80" value="25">
            <span class="val" id="speedVal">25</span>
        </div>
    </div>

    <div class="card">
        <h2>Arm</h2>
        <div class="grid-3" style="margin-bottom:10px;">
            <button data-cmd="ARM" data-arg="HOME" class="danger">Home</button>
            <button data-cmd="ARM" data-arg="SCAN_POSE">Scan Pose</button>
            <button data-cmd="ARM" data-arg="CTP">Car &rarr; Plt</button>
        </div>
        <div class="section-label">To Platform</div>
        <div class="grid-3">
            <button data-cmd="ARM" data-arg="RTP" class="btn-r">R &rarr; Plt</button>
            <button data-cmd="ARM" data-arg="GTP" class="btn-g">G &rarr; Plt</button>
            <button data-cmd="ARM" data-arg="BTP" class="btn-b">B &rarr; Plt</button>
        </div>
        <div class="section-label" style="margin-top:8px">To Car</div>
        <div class="grid-3">
            <button data-cmd="ARM" data-arg="RTC" class="btn-r">R &rarr; Car</button>
            <button data-cmd="ARM" data-arg="GTC" class="btn-g">G &rarr; Car</button>
            <button data-cmd="ARM" data-arg="BTC" class="btn-b">B &rarr; Car</button>
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
        <button id="btnAuto" class="btn-auto" data-cmd="AUTO" data-arg="TOGGLE">START AUTONOMOUS</button>
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
                    $('btnAuto').classList.add('active');
                    $('btnAuto').textContent='RUNNING...';
                }else{
                    $('btnAuto').classList.remove('active');
                    $('btnAuto').textContent='START AUTONOMOUS';
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
                    $('camDist').textContent=qr.tz_mm!=null?parseFloat(qr.tz_mm).toFixed(0)+'mm':'--';
                    $('camYaw').textContent=qr.yaw_deg!=null?parseFloat(qr.yaw_deg).toFixed(1)+'\u00B0':'--';
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

        document.querySelectorAll('button[data-cmd]').forEach(function(b){
            if(b.closest('.pad')||b.id==='btnScanQR')return;
            b.addEventListener('click',function(){send({cmd:b.dataset.cmd,arg:b.dataset.arg})});
        });
        $('btnAuto').addEventListener('click',function(){$('btnAuto').classList.toggle('active')});
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
// SETUP
// ================================================================

void setup() {
  Serial.begin(115200);

  // I2C: Start at 400kHz for MPU6050 DMP init
  Wire.begin(SDA_PIN, SCL_PIN, 400000);
  Wire.setTimeOut(200);
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

  // Switch to 40kHz for motor driver
  Wire.setClock(40000);

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

  // Add Arm peer
  esp_now_peer_info_t armPeer = {};
  memcpy(armPeer.peer_addr, armAddress, 6);
  armPeer.channel = WIFI_CHANNEL;
  armPeer.encrypt = false;
  armPeer.ifidx = WIFI_IF_AP;

  if (esp_now_add_peer(&armPeer) != ESP_OK) {
    Serial.println("Failed to Add Arm Peer");
  } else {
    Serial.println("Arm ESP-NOW Peer Added");
  }

  // Add Camera peer
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

  Serial.println("ESP-NOW READY");

  // ================= WEB SERVER =================

  // Serve controller page at root
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", CONTROLLER_HTML);
  });

  // Status endpoint — polled by GUI every 500ms
  server.on("/status", HTTP_GET, []() {
    String json = "{";
    json += "\"arm_busy\":" + String(lastArmStatus.busy ? "true" : "false");
    json += ",\"autonomous\":" + String(autonomousMode ? "true" : "false");
    json += ",\"qr_pending\":" + String(newQrResult ? "false" : "true");
    json += ",\"scan_in_progress\":" + String(scanInProgress ? "true" : "false");
    json += ",\"scan_status\":\"" + String(scanInProgress ? "Accumulating" : "DONE") + "\"";
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
      json += ",\"scan_status\":\"" + String(lastScanStatus == 1 ? "DONE" : "Accumulating") + "\"";
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

  server.begin();
  Serial.println("Web server started on port 80");
  Serial.printf("Open http://%s in phone browser\n",
                WiFi.softAPIP().toString().c_str());
}

// ================================================================
// LOOP
// ================================================================

void loop() {
  server.handleClient();

  // =================== CONTINUOUS MOVEMENT ===================
  if (!autonomousMode) updateMotors();

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
        reply.pose_valid, reply.color, reply.confidence,
        reply.yaw_deg, reply.tz_mm, reply.estimated);
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
    // Camera still accumulating — reset watchdog so base doesn't timeout mid-retry
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

  // =================== STEP HANDLER ===================
  if (pendingStep && !moveActive) {
    float stepDist = 0.05;
    float stepDeg = 15.0;
    if (stepArg == "FWD") {
      moveDistanceKp(V_FORWARD, Motor_speed, stepDist, TICKS_FWD_BWD);
    } else if (stepArg == "BACK") {
      moveDistanceKp(V_BACKWARD, Motor_speed, stepDist, TICKS_FWD_BWD);
    } else if (stepArg == "LEFT") {
      moveDistanceKp(V_STRAFE_L, Motor_speed, stepDist, TICKS_STRAFE);
    } else if (stepArg == "RIGHT") {
      moveDistanceKp(V_STRAFE_R, Motor_speed, stepDist, TICKS_STRAFE);
    } else if (stepArg == "ROTCW") {
      rotateDegrees(true, stepDeg, Motor_speed);
    } else if (stepArg == "ROTCCW") {
      rotateDegrees(false, stepDeg, Motor_speed);
    } else if (stepArg == "DIAGFR") {
      moveDistanceKp(V_DIAG_FR, Motor_speed, stepDist, TICKS_DIAG);
    } else if (stepArg == "DIAGFL") {
      moveDistanceKp(V_DIAG_FL, Motor_speed, stepDist, TICKS_DIAG);
    } else if (stepArg == "DIAGBR") {
      moveDistanceKp(V_DIAG_BR, Motor_speed, stepDist, TICKS_DIAG);
    } else if (stepArg == "DIAGBL") {
      moveDistanceKp(V_DIAG_BL, Motor_speed, stepDist, TICKS_DIAG);
    }
    pendingStep = false;
  }

  // =================== ESP-NOW DEBUG ===================
  static unsigned long lastStatusUpdate = 0;
  if (millis() - lastStatusUpdate > 5000) {
    lastStatusUpdate = millis();
    Serial.printf("[ESP-NOW RX] total=%lu bytes=%lu pose=%lu arm=%lu nomatch=%lu\n",
      (unsigned long)espNowRecvCount, (unsigned long)espNowRecvBytes,
      (unsigned long)espNowRecvMatchPose, (unsigned long)espNowRecvMatchArm,
      (unsigned long)espNowRecvNoMatch);
  }

  // =================== AUTONOMOUS SEQUENCE ===================
  // Fixed competition sequence — runs to completion once started.
  // To modify: change distances (meters), arm commands, or step order.
  // Each step: movement (vector + distance in meters) or arm action (command + wait).
  // Sequence: FWD 1m -> Red->Plat -> LEFT 1m -> BACK 1m -> Blue->Plat
  //           -> RIGHT 1m -> Green->Plat -> DIAG 1m
  // Toggle autonomous OFF to stop mid-sequence.
  if (autonomousMode && autoTrigger == 1) {
    autoTrigger = 0;
    Serial.println("[AUTO] === Starting fixed autonomous sequence ===");

    // --- Step 1: Move FORWARD 1 meter ---
    Serial.println("[AUTO] Step 1: Forward 1m");
    moveDistanceKp(V_FORWARD, Motor_speed, 1.0, TICKS_FWD_BWD);
    if (!autonomousMode) goto autoEnd;

    // --- Step 2: Drop RED box on platform ---
    // Arm: pick from red pos, transit home (gripper closed), place at platform
    Serial.println("[AUTO] Step 2: Red -> Platform");
    sendCommandToArm("RTP");
    waitForArmIdle(20000);
    if (!autonomousMode) goto autoEnd;

    // --- Step 3: Move LEFT 1 meter ---
    Serial.println("[AUTO] Step 3: Left 1m");
    moveDistanceKp(V_STRAFE_L, Motor_speed, 1.0, TICKS_STRAFE);
    if (!autonomousMode) goto autoEnd;

    // --- Step 4: Move BACKWARD 1 meter ---
    Serial.println("[AUTO] Step 4: Backward 1m");
    moveDistanceKp(V_BACKWARD, Motor_speed, 1.0, TICKS_FWD_BWD);
    if (!autonomousMode) goto autoEnd;

    // --- Step 5: Drop BLUE box on platform ---
    Serial.println("[AUTO] Step 5: Blue -> Platform");
    sendCommandToArm("BTP");
    waitForArmIdle(20000);
    if (!autonomousMode) goto autoEnd;

    // --- Step 6: Move RIGHT 1 meter ---
    Serial.println("[AUTO] Step 6: Right 1m");
    moveDistanceKp(V_STRAFE_R, Motor_speed, 1.0, TICKS_STRAFE);
    if (!autonomousMode) goto autoEnd;

    // --- Step 7: Drop GREEN box on platform ---
    Serial.println("[AUTO] Step 7: Green -> Platform");
    sendCommandToArm("GTP");
    waitForArmIdle(20000);
    if (!autonomousMode) goto autoEnd;

    // --- Step 8: Move DIAGONAL (forward-right) 1 meter ---
    Serial.println("[AUTO] Step 8: Diagonal 1m");
    moveDistanceKp(V_DIAG_FR, Motor_speed, 1.0, TICKS_DIAG);

    Serial.println("[AUTO] === Sequence complete ===");
    autoEnd:
    forceStop();
    autonomousMode = false;
    Serial.println("[AUTO] Autonomous sequence ended");
  }
}
