// =========================== BASE ESP32 (WiFi AP + WebSocket + ESP-NOW) ===========================
// Phone → WiFi "GLITCH" → http://192.168.4.1 → WebSocket → ESP-NOW → Arm
// Combines: old text/JSON WS protocol + camera + autonomous + telemetry
//           new WiFi stability settings + cleanupClients + channel 11

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <esp_wifi.h>
#include <math.h>
#include <Wire.h>
#include <esp_now.h>

// ================= WIFI AP =================
const char* AP_SSID = "GLITCH";
const char* AP_PASS = "Gl1tch2024!Secure";
const uint8_t WIFI_CHANNEL = 11;

// ================= WEB SERVER =================
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

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
static uint8_t cameraAddress[] = {0x94, 0xA9, 0x90, 0x08, 0xB2, 0xB8};

// ESP-NOW packet: Base → Camera (scan request)
struct __attribute__((packed)) ScanRequest {
  uint8_t task_id;
  uint8_t mode; // 0=scan_qr, 1=scan_platform
  uint8_t reserved[2];
};

// ESP-NOW packet: Camera → Base (pose reply)
struct __attribute__((packed)) PoseReply {
  uint8_t task_id;
  uint8_t pose_valid;
  uint8_t color; // 0=unknown, 1=R, 2=G, 3=B
  uint8_t estimated;
  float tx_mm;
  float ty_mm;
  float tz_mm;
  float yaw_deg;
  float confidence;
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

// --- STEP MOVEMENT FLAG ---
bool pendingStep = false;
String stepArg = "";

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

// --- CALIBRATED TICK CONSTANTS (WEIGHT-AWARE) ---
const float TICKS_FWD_BWD = 5540.0;
const float TICKS_STRAFE = 6253.0;
const float TICKS_DIAG = 7875.0;
const float TICKS_ROTATE = 7200.0;

int8_t Motor_speed = 25;

// --- STABILITY CONTROL ---
const float KP_POS = 0.005;
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

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("ESP-NOW Send Status: ");
  if (status == ESP_NOW_SEND_SUCCESS)
    Serial.println("SUCCESS");
  else
    Serial.println("FAILED");
}

// ESP-NOW receive callback — handles pose replies from camera + status from arm
void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len == sizeof(PoseReply)) {
    memcpy((void *)&lastPoseReply, data, sizeof(PoseReply));
    cameraPoseReceived = true;

    Serial.printf(
        "[CAM] Pose: valid=%d color=%d conf=%.2f yaw=%.1f tz=%.1f est=%d\n",
        lastPoseReply.pose_valid, lastPoseReply.color, lastPoseReply.confidence,
        lastPoseReply.yaw_deg, lastPoseReply.tz_mm, lastPoseReply.estimated);
  } else if (len == sizeof(ArmStatus)) {
    ArmStatus st;
    memcpy(&st, data, sizeof(st));
    String json = "{\"type\":\"arm_status\",\"busy\":" + String(st.busy ? "true" : "false") + "}";
    ws.textAll(json);
    Serial.printf("[ARM] Status: busy=%d\n", st.busy);
  }
}

void sendCommandToArm(const char *cmd) {
  strcpy(armMessage.command, cmd);
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
  req.task_id = ++taskCounter;
  req.mode = mode;
  esp_err_t result = esp_now_send(cameraAddress, (uint8_t *)&req, sizeof(req));
  Serial.printf("[CAM] Scan request sent: task=%d mode=%d (%s)\n", req.task_id,
                req.mode, result == ESP_OK ? "OK" : "FAILED");
}

void sendCameraPoseToArm() {
  CameraPoseData data = {};
  data.type = 1;
  data.pose_valid = lastPoseReply.pose_valid;
  data.color = lastPoseReply.color;
  data.estimated = lastPoseReply.estimated;
  data.tx_mm = lastPoseReply.tx_mm;
  data.ty_mm = lastPoseReply.ty_mm;
  data.tz_mm = lastPoseReply.tz_mm;
  data.yaw_deg = lastPoseReply.yaw_deg;
  data.confidence = lastPoseReply.confidence;

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
  int8_t speeds[4] = {v1, v2, v3, v4};
  return writeBytes(REG_FIXED_SPEED, (uint8_t *)speeds, 4);
}

void manualMove(const int8_t vector[], int8_t speedVal) {
  for (int i = 0; i < 5; i++) {
    if (writeSpeeds(vector[0] * speedVal, vector[1] * speedVal,
                    vector[2] * speedVal, vector[3] * speedVal)) {
      break;
    }
    delay(5);
  }
}

void forceStop() {
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
// MOVEMENT LOGIC
// ================================================================

void moveDistanceKp(const int8_t vector[], int8_t maxSpeed, float distance,
                    float tickConstant) {
  int32_t startEncoders[4], currentEncoders[4];
  long targetTicks = lroundf(fabs(distance * tickConstant));
  int8_t localMinTorque = (tickConstant == TICKS_ROTATE) ? 22 : MIN_TORQUE;
  maxSpeed = max(maxSpeed, localMinTorque);
  if (!readEncoders(startEncoders))
    return;
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
    int8_t finalSpeed;
    if (error < BRAKE_ZONE_TICKS)
      finalSpeed = localMinTorque;
    else
      finalSpeed = (int8_t)constrain(calcSpeed, localMinTorque, maxSpeed);
    if (finalSpeed != lastSpeed) {
      writeSpeeds(finalSpeed * vector[0], finalSpeed * vector[1],
                  finalSpeed * vector[2], finalSpeed * vector[3]);
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
  const int8_t *vector = clockwise ? V_ROTATE_CW : V_ROTATE_CCW;
  float distanceFraction = degrees / 360.0;
  moveDistanceKp(vector, maxSpeed, distanceFraction, TICKS_ROTATE);
}

// ================================================================
// AUTONOMOUS CAMERA ALIGNMENT
// ================================================================

static bool waitForCameraPose(unsigned long timeout_ms) {
  cameraPoseReceived = false;
  sendScanRequest(0);
  unsigned long t0 = millis();
  while (!cameraPoseReceived && millis() - t0 < timeout_ms) {
    if (!autonomousMode)
      return false;
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

static bool alignToQR() {
  const float YAW_THRESHOLD_DEG = 6.0f;
  const float APPROACH_DISTANCE_MM = 200.0f;
  const float CONFIDENCE_THRESHOLD = 0.55f;
  const int ALIGN_SPEED = 20;
  const int MAX_ALIGN_STEPS = 12;
  const int MAX_APPROACH_STEPS = 10;

  if (!waitForCameraPose(5000))
    return false;

  Serial.printf("[AUTO] Detected: color=%d conf=%.2f yaw=%.1f dist=%.0fmm\n",
                lastPoseReply.color, lastPoseReply.confidence,
                lastPoseReply.yaw_deg, lastPoseReply.tz_mm);

  for (int step = 0; step < MAX_ALIGN_STEPS; step++) {
    if (!autonomousMode)
      return false;
    if (fabs(lastPoseReply.yaw_deg) <= YAW_THRESHOLD_DEG)
      break;
    if (lastPoseReply.yaw_deg > 0)
      manualMove(V_STRAFE_R, ALIGN_SPEED);
    else
      manualMove(V_STRAFE_L, ALIGN_SPEED);
    delay(120);
    forceStop();
    if (!waitForCameraPose(3000))
      return false;
  }
  forceStop();
  Serial.printf("[AUTO] Yaw aligned: %.1f°\n", lastPoseReply.yaw_deg);

  for (int step = 0; step < MAX_APPROACH_STEPS; step++) {
    if (!autonomousMode)
      return false;
    if (lastPoseReply.tz_mm <= APPROACH_DISTANCE_MM)
      break;
    manualMove(V_FORWARD, ALIGN_SPEED);
    delay(200);
    forceStop();
    if (!waitForCameraPose(3000))
      return false;
  }
  forceStop();
  Serial.printf("[AUTO] Approach complete: dist=%.0fmm conf=%.2f\n",
                lastPoseReply.tz_mm, lastPoseReply.confidence);

  if (lastPoseReply.confidence < CONFIDENCE_THRESHOLD) {
    Serial.println("[AUTO] Low confidence, aborting pickup");
    return false;
  }
  return true;
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

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len &&
      info->opcode == WS_TEXT) {

    char buf[256];
    size_t copyLen = (len < sizeof(buf) - 1) ? len : sizeof(buf) - 1;
    memcpy(buf, data, copyLen);
    buf[copyLen] = 0;
    String msg = buf;
    Serial.printf("[WS] Received: %s\n", msg.c_str());

    String cmd = jsonStr(msg, "cmd");
    String arg = jsonStr(msg, "arg");

    if (cmd == "MOVE") {
      if (!autonomousMode) {
        if (arg == "STOP") {
          forceStop();
          return;
        }
        if (arg == "FWD")
          manualMove(V_FORWARD, Motor_speed);
        else if (arg == "BACK")
          manualMove(V_BACKWARD, Motor_speed);
        else if (arg == "LEFT")
          manualMove(V_STRAFE_L, Motor_speed);
        else if (arg == "RIGHT")
          manualMove(V_STRAFE_R, Motor_speed);
        else if (arg == "ROTCW")
          manualMove(V_ROTATE_CW, Motor_speed);
        else if (arg == "ROTCCW")
          manualMove(V_ROTATE_CCW, Motor_speed);
        else if (arg == "DIAGFR")
          manualMove(V_DIAG_FR, Motor_speed);
        else if (arg == "DIAGFL")
          manualMove(V_DIAG_FL, Motor_speed);
        else if (arg == "DIAGBR")
          manualMove(V_DIAG_BR, Motor_speed);
        else if (arg == "DIAGBL")
          manualMove(V_DIAG_BL, Motor_speed);
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
      } else {
        sendCommandToArm(arg.c_str());
      }
    } else if (cmd == "SSTEP") {
      // arg = "J1,5" or "J3,-10" — single-step servo
      int joint = arg.charAt(1) - '1'; // J1=0, J2=1, ...
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
        sendScanRequest(0);
        Serial.println("[SCAN] QR scan requested");
      } else if (arg == "PLAT") {
        sendScanRequest(1);
        Serial.println("[SCAN] Platform scan requested");
      }
    } else if (cmd == "SERVO") {
      int idx = arg.substring(0, arg.indexOf(':')).toInt();
      String dirStr = arg.substring(arg.indexOf(':') + 1);
      int dir = (dirStr == "UP" || dirStr == "1") ? 1 : -1;
      servoStep(idx, dir);
    }
  }
}

// ================================================================
// WEBSOCKET EVENT HANDLER
// ================================================================

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("[WS] Client #%u connected from %s\n", client->id(),
                     client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("[WS] Client #%u disconnected\n", client->id());
      forceStop();
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

// ================================================================
// TELEMETRY PUSH
// ================================================================

void sendTelemetry() {
  if (ws.count() == 0) return;

  String json = "{\"type\":\"telemetry\"";
  json += ",\"confidence\":" + String(lastPoseReply.confidence, 2);
  json += ",\"yaw\":" + String(lastPoseReply.yaw_deg, 1);
  json += ",\"color\":\"" + String(colorName(lastPoseReply.color)) + "\"";
  json += ",\"distance_mm\":" + String(lastPoseReply.tz_mm, 0);
  json += ",\"motor_speed\":" + String(Motor_speed);
  json += ",\"free_heap\":" + String(ESP.getFreeHeap());
  json += ",\"autonomous\":" + String(autonomousMode ? "true" : "false");
  json += ",\"connected\":true";
  json += "}";

  ws.textAll(json);
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
    </style>
</head>
<body>
    <h1>Glitch</h1>
    <div class="sub" id="connStatus"><span class="dot"></span><span>connecting…</span></div>

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
        <h2>Arm Commands</h2>
        <div class="grid-2">
            <button data-cmd="ARM" data-arg="GTF">Green &rarr; Floor</button>
            <button data-cmd="ARM" data-arg="BTC">Blue &rarr; Car</button>
            <button data-cmd="ARM" data-arg="RTC">Red &rarr; Car</button>
            <button data-cmd="ARM" data-arg="GTC">Green &rarr; Car</button>
            <button data-cmd="ARM" data-arg="HOME" class="danger">Home</button>
            <button data-cmd="ARM" data-arg="RTF">Red &rarr; Floor</button>
            <button data-cmd="ARM" data-arg="BTF">Blue &rarr; Floor</button>
            <button data-cmd="ARM" data-arg="CTP">Car &rarr; Platform</button>
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
        <div class="grid-2">
            <button id="btnAuto" data-cmd="AUTO" data-arg="TOGGLE">Autonomous</button>
            <button data-cmd="SCAN" data-arg="QR">Scan QR</button>
            <button data-cmd="SCAN" data-arg="PLAT">Scan Platform</button>
            <button data-cmd="ARM" data-arg="SCAN_POSE">Arm Scan Pose</button>
        </div>
    </div>

    <div class="card">
        <h2>Event Log</h2>
        <div class="log" id="log"></div>
    </div>

    <script>
        const $ = (id) => document.getElementById(id);
        const log = (msg, cls='') => {
            const el = $('log');
            const t = new Date().toLocaleTimeString();
            el.innerHTML = '<div class="' + cls + '">[' + t + '] ' + msg + '</div>' + el.innerHTML;
            while (el.children.length > 80) el.removeChild(el.lastChild);
        };
        const setStatus = (cls, text) => {
            $('connStatus').innerHTML = '<span class="dot ' + cls + '"></span><span>' + text + '</span>';
        };

        // ── Arm busy state ─────────────────────────────────────────
        let armBusy = false;
        function setArmBusy(busy) {
            armBusy = busy;
            document.querySelectorAll('.step-btn').forEach(b => b.disabled = busy);
            log(busy ? 'Arm: busy' : 'Arm: idle', busy ? 'err' : 'ok');
        }

        // ── Native WebSocket ───────────────────────────────────────
        let ws = null;
        let reconnectTimer = null;

        function connect() {
            const host = location.hostname || '192.168.4.1';
            ws = new WebSocket('ws://' + host + '/ws');

            ws.onopen = () => {
                setStatus('ok', 'connected to base');
                log('WebSocket connected', 'ok');
                if (reconnectTimer) { clearInterval(reconnectTimer); reconnectTimer = null; }
            };

            ws.onclose = () => {
                setStatus('', 'disconnected');
                log('WebSocket disconnected', 'err');
                if (!reconnectTimer) {
                    reconnectTimer = setInterval(() => {
                        log('Reconnecting...', '');
                        connect();
                    }, 3000);
                }
            };

            ws.onerror = () => { ws.close(); };

            ws.onmessage = (evt) => {
                try {
                    const s = JSON.parse(evt.data);
                    if (s.type === 'arm_status') {
                        setArmBusy(s.busy);
                    }
                } catch(e) {}
            };
        }

        connect();

        const send = (data) => {
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify(data));
            }
        };

        // ── D-pad & Rotation: tap to move, tap STOP to stop ────────
        const driveBtns = document.querySelectorAll('.pad button[data-cmd="MOVE"], #btnRotCCW, #btnRotCW');

        const sendDriveCmd = function(e) {
            e.preventDefault();
            driveBtns.forEach(btn => btn.classList.remove('active'));
            this.classList.add('active');
            send({cmd:'MOVE', arg: this.dataset.arg});
        };

        driveBtns.forEach(b => {
            b.addEventListener('touchstart', sendDriveCmd, { passive: false });
            b.addEventListener('mousedown', sendDriveCmd);
        });

        // ── Single-shot buttons ────────────────────────────────────
        document.querySelectorAll('button[data-cmd]').forEach(b => {
            if (b.closest('.pad')) return;
            b.addEventListener('click', () => send({cmd:b.dataset.cmd, arg:b.dataset.arg}));
        });
        $('btnAuto').addEventListener('click', () => $('btnAuto').classList.toggle('active'));

        // ── Speed slider (debounced) ───────────────────────────────
        let speedT;
        $('speed').addEventListener('input', (e) => { $('speedVal').textContent = e.target.value; });
        $('speed').addEventListener('input', (e) => {
            clearTimeout(speedT);
            speedT = setTimeout(() => send({cmd:'SPEED', arg: parseInt(e.target.value)}), 120);
        });

        // ── Step servo buttons ─────────────────────────────────────
        const stepJoints = ['J1','J2','J3','J4','J5'];
        document.querySelectorAll('.step-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                if (armBusy) return;
                const j = parseInt(btn.dataset.joint);
                const dir = btn.dataset.dir === '+' ? 1 : -1;
                const deg = parseInt($('stepSize').value) || 5;
                const val = dir * deg;
                send({cmd:'SSTEP', arg: stepJoints[j] + ',' + val});
            });
        });

        // ── Keyboard arrows for laptop use ─────────────────────────
        const keyMap = { ArrowUp:'FWD', ArrowDown:'BACK', ArrowLeft:'LEFT', ArrowRight:'RIGHT' };
        let keysDown = new Set();
        document.addEventListener('keydown', e => {
            if (keyMap[e.key] && !keysDown.has(e.key)) {
                keysDown.add(e.key); send({cmd:'MOVE', arg: keyMap[e.key]});
            }
            if (e.key === ' ') { e.preventDefault(); send({cmd:'MOVE', arg:'STOP'}); }
        });
        document.addEventListener('keyup', e => {
            if (keyMap[e.key]) { keysDown.delete(e.key); send({cmd:'MOVE', arg:'STOP'}); }
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

  // I2C: 40 kHz (matches working Blynk config)
  Wire.begin(SDA_PIN, SCL_PIN, 40000);
  Wire.setTimeOut(200);
  delay(500);

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
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS, WIFI_CHANNEL);
  delay(100);
  esp_wifi_set_ps(WIFI_PS_NONE);

  Serial.println("\n========================================");
  Serial.printf("  WiFi AP: %s\n", AP_SSID);
  Serial.printf("  IP: %s\n", WiFi.softAPIP().toString().c_str());
  Serial.printf("  BASE MAC: %s\n", WiFi.macAddress().c_str());
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
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // Serve controller page at root
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", CONTROLLER_HTML);
  });

  // Health check endpoint
  server.on("/healthz", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = "{\"ok\":true,\"clients\":" + String(ws.count()) + "}";
    request->send(200, "application/json", json);
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
  ws.cleanupClients();

  // =================== STEP HANDLER ===================
  if (pendingStep) {
    float stepDist = 50.0;
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

  // =================== TELEMETRY PUSH ===================
  static unsigned long lastTelemetry = 0;
  if (millis() - lastTelemetry > 2000) {
    lastTelemetry = millis();
    sendTelemetry();
  }

  // =================== AUTONOMOUS MODE ===================
  if (autonomousMode && autoTrigger == 1) {
    Serial.println("[AUTO] Starting camera-guided autonomous pickup");

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
      if (!autonomousMode)
        break;
      Serial.printf("[AUTO] --- Target %d: %s ---\n", i + 1, targets[i].label);

      if (!alignToQR()) {
        Serial.printf("[AUTO] Alignment failed for %s, skipping\n",
                      targets[i].label);
        continue;
      }

      if (!autonomousMode)
        break;

      if (lastPoseReply.color == targets[i].colorCode) {
        Serial.printf("[AUTO] Color match: %s — sending camera pose to arm\n",
                      targets[i].label);
        sendCameraPoseToArm();

        unsigned long t0 = millis();
        while (millis() - t0 < 12000) {
          ws.cleanupClients();
          if (!autonomousMode)
            break;
          delay(50);
        }
      } else {
        const char *cn[] = {"NONE", "RED", "GREEN", "BLUE"};
        int ci = lastPoseReply.color;
        if (ci > 3) ci = 0;
        Serial.printf(
            "[AUTO] Color mismatch: expected %s, camera sees %s — skipping\n",
            targets[i].label, cn[ci]);
      }
    }

    if (autonomousMode)
      sendCommandToArm("H");
    forceStop();
    autoTrigger = 0;
    autonomousMode = false;
    Serial.println("[AUTO] Autonomous pickup complete");
  }
}
