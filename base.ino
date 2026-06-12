// =========================== BASE ESP32 CODE ===========================
// WiFi AP + WebSocket controller — no laptop, no Blynk, no Flask.
// Phone connects to "GLITCH" WiFi → opens http://192.168.4.1

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <math.h>
#include <Wire.h>
#include <esp_now.h>

// ================= WIFI AP =================
const char* AP_SSID = "GLITCH";
const char* AP_PASS = "12345678";

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

// Latest camera pose data (updated by ESP-NOW callback)
static volatile bool cameraPoseReceived = false;
static volatile PoseReply lastPoseReply;

// Movement watchdog
static bool isMoving = false;
static unsigned long lastMoveTime = 0;
const unsigned long MOVE_TIMEOUT_MS = 3000;

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

// ESP-NOW receive callback — handles pose replies from camera
void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len == sizeof(PoseReply)) {
    memcpy((void *)&lastPoseReply, data, sizeof(PoseReply));
    cameraPoseReceived = true;

    Serial.printf(
        "[CAM] Pose: valid=%d color=%d conf=%.2f yaw=%.1f tz=%.1f est=%d\n",
        lastPoseReply.pose_valid, lastPoseReply.color, lastPoseReply.confidence,
        lastPoseReply.yaw_deg, lastPoseReply.tz_mm, lastPoseReply.estimated);
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
// HELPER FUNCTIONS
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

void writeSpeeds(int8_t v1, int8_t v2, int8_t v3, int8_t v4) {
  int8_t speeds[4] = {v1, v2, v3, v4};
  writeBytes(REG_FIXED_SPEED, (uint8_t *)speeds, 4);
}

void manualMove(const int8_t vector[], int8_t speedVal) {
  writeSpeeds(vector[0] * speedVal, vector[1] * speedVal, vector[2] * speedVal,
              vector[3] * speedVal);
}

void forceStop() { writeSpeeds(0, 0, 0, 0); }

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
// WEBSOCKET COMMAND HANDLER
// ================================================================

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len &&
      info->opcode == WS_TEXT) {

    data[len] = 0; // null terminate
    String msg = (char *)data;
    Serial.printf("[WS] Received: %s\n", msg.c_str());

    // Minimal JSON parser — extract "cmd" and "arg"
    String cmd = jsonStr(msg, "cmd");
    String arg = jsonStr(msg, "arg");

    if (cmd == "MOVE") {
      if (!autonomousMode) {
        if (arg == "FWD") {
          manualMove(V_FORWARD, Motor_speed);
          isMoving = true; lastMoveTime = millis();
        } else if (arg == "BACK") {
          manualMove(V_BACKWARD, Motor_speed);
          isMoving = true; lastMoveTime = millis();
        } else if (arg == "LEFT") {
          manualMove(V_STRAFE_L, Motor_speed);
          isMoving = true; lastMoveTime = millis();
        } else if (arg == "RIGHT") {
          manualMove(V_STRAFE_R, Motor_speed);
          isMoving = true; lastMoveTime = millis();
        } else if (arg == "ROTCW") {
          manualMove(V_ROTATE_CW, Motor_speed);
          isMoving = true; lastMoveTime = millis();
        } else if (arg == "ROTCCW") {
          manualMove(V_ROTATE_CCW, Motor_speed);
          isMoving = true; lastMoveTime = millis();
        } else if (arg == "DIAGFR") {
          manualMove(V_DIAG_FR, Motor_speed);
          isMoving = true; lastMoveTime = millis();
        } else if (arg == "DIAGFL") {
          manualMove(V_DIAG_FL, Motor_speed);
          isMoving = true; lastMoveTime = millis();
        } else if (arg == "DIAGBR") {
          manualMove(V_DIAG_BR, Motor_speed);
          isMoving = true; lastMoveTime = millis();
        } else if (arg == "DIAGBL") {
          manualMove(V_DIAG_BL, Motor_speed);
          isMoving = true; lastMoveTime = millis();
        } else { // STOP
          forceStop();
          isMoving = false;
        }
      }
    } else if (cmd == "STEP") {
      if (!autonomousMode) {
        float stepDist = 50.0; // 50mm
        float stepDeg = 15.0; // 15 degrees
        if (arg == "FWD") {
          moveDistanceKp(V_FORWARD, Motor_speed, stepDist, TICKS_FWD_BWD);
        } else if (arg == "BACK") {
          moveDistanceKp(V_BACKWARD, Motor_speed, stepDist, TICKS_FWD_BWD);
        } else if (arg == "LEFT") {
          moveDistanceKp(V_STRAFE_L, Motor_speed, stepDist, TICKS_STRAFE);
        } else if (arg == "RIGHT") {
          moveDistanceKp(V_STRAFE_R, Motor_speed, stepDist, TICKS_STRAFE);
        } else if (arg == "ROTCW") {
          rotateDegrees(true, stepDeg, Motor_speed);
        } else if (arg == "ROTCCW") {
          rotateDegrees(false, stepDeg, Motor_speed);
        } else if (arg == "DIAGFR") {
          moveDistanceKp(V_DIAG_FR, Motor_speed, stepDist, TICKS_DIAG);
        } else if (arg == "DIAGFL") {
          moveDistanceKp(V_DIAG_FL, Motor_speed, stepDist, TICKS_DIAG);
        } else if (arg == "DIAGBR") {
          moveDistanceKp(V_DIAG_BR, Motor_speed, stepDist, TICKS_DIAG);
        } else if (arg == "DIAGBL") {
          moveDistanceKp(V_DIAG_BL, Motor_speed, stepDist, TICKS_DIAG);
        }
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
        // Direct arm command (BTC, RTC, GTC, RTF, BTF, GTF, etc.)
        sendCommandToArm(arg.c_str());
      }
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
      isMoving = false;
    } else if (cmd == "SCAN") {
      if (arg == "QR") {
        sendScanRequest(0);
        Serial.println("[SCAN] QR scan requested");
      } else if (arg == "PLAT") {
        sendScanRequest(1);
        Serial.println("[SCAN] Platform scan requested");
      }
    } else if (cmd == "SERVO") {
      // arg format: "0:1" (idx:dir) or "0:UP"/"0:DOWN"
      int idx = arg.substring(0, arg.indexOf(':')).toInt();
      String dirStr = arg.substring(arg.indexOf(':') + 1);
      int dir = (dirStr == "UP" || dirStr == "1") ? 1 : -1;
      servoStep(idx, dir);
    }
  }
}

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
      // Safety: stop motors when phone disconnects
      forceStop();
      isMoving = false;
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

// The controller page is served from the dashboard/controller.html file.
// For robustness on ESP32, we embed a minimal redirect if SPIFFS isn't used.
// The actual page is loaded via the /controller.html route below.

// ================================================================
// SETUP
// ================================================================

extern const char CONTROLLER_HTML[];

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
  Serial.println("Motor System Ready");

  // ================= WIFI AP =================
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(100);

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
  armPeer.channel = WiFi.channel();
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
  camPeer.channel = WiFi.channel();
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

  // =================== MOVEMENT WATCHDOG ===================
  if (isMoving && (millis() - lastMoveTime > MOVE_TIMEOUT_MS)) {
    forceStop();
    isMoving = false;
    Serial.println("[WATCHDOG] Auto-stop: move timeout");
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

// ================================================================
// EMBEDDED CONTROLLER HTML
// ================================================================
// This is the full controller.html embedded as a raw string literal.
// It uses native WebSocket (no CDN/internet dependency).

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
        .pad .empty { visibility: hidden; }
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
        <h2>Mode</h2>
        <div class="grid-2">
            <button id="btnAuto" data-cmd="AUTO" data-arg="TOGGLE">Autonomous</button>
            <button data-cmd="SCAN" data-arg="QR">Scan QR</button>
            <button data-cmd="SCAN" data-arg="PLAT">Scan Platform</button>
            <button data-cmd="ARM" data-arg="SCAN_POSE">Arm Scan Pose</button>
        </div>
    </div>

    <div class="card">
        <h2>Live Telemetry</h2>
        <div class="telemetry">
            <div class="tile"><div class="label">Confidence</div><div class="val" id="tConf">&mdash;</div></div>
            <div class="tile"><div class="label">Yaw &deg;</div><div class="val" id="tYaw">&mdash;</div></div>
            <div class="tile"><div class="label">Color</div><div class="val" id="tColor">&mdash;</div></div>
            <div class="tile"><div class="label">Distance mm</div><div class="val" id="tDist">&mdash;</div></div>
            <div class="tile"><div class="label">Motor Speed</div><div class="val" id="tSpeed">&mdash;</div></div>
            <div class="tile"><div class="label">Heap / State</div><div class="val small" id="tHeap">&mdash;</div></div>
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

        // ── Native WebSocket to Base ESP32 ─────────────────────────
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

            ws.onerror = () => {
                ws.close();
            };

            ws.onmessage = (evt) => {
                try {
                    const s = JSON.parse(evt.data);
                    if (s.type === 'telemetry') {
                        $('tConf').textContent  = (s.confidence*100).toFixed(0) + '%';
                        $('tYaw').textContent   = parseFloat(s.yaw).toFixed(1);
                        $('tColor').textContent = s.color;
                        $('tDist').textContent  = parseInt(s.distance_mm);
                        $('tSpeed').textContent = s.motor_speed;
                        $('tHeap').textContent  = (s.free_heap/1024).toFixed(0) + 'kB / ' + (s.autonomous ? 'AUTO' : 'MANUAL');
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

        // ── D-pad & Rotation: Smart tap vs hold ───────────────────
        const driveBtns = document.querySelectorAll('.pad button[data-cmd="MOVE"], #btnRotCCW, #btnRotCW');
        driveBtns.forEach(b => {
            let pressTimer = null;
            let isHolding = false;
            
            const start = (e) => { 
                e.preventDefault(); 
                if (b.dataset.arg === 'STOP') {
                    b.classList.add('active'); 
                    send({cmd:'MOVE', arg:'STOP'});
                    return;
                }
                b.classList.add('active');
                isHolding = false;
                
                // Start timer for hold vs tap
                pressTimer = setTimeout(() => {
                    isHolding = true;
                    send({cmd:'MOVE', arg:b.dataset.arg});
                }, 300); // 300ms threshold
            };
            
            const end = (e) => { 
                e.preventDefault(); 
                b.classList.remove('active'); 
                if (b.dataset.arg === 'STOP') return;
                
                if (pressTimer) clearTimeout(pressTimer);
                
                if (isHolding) {
                    // It was a long press, send stop
                    send({cmd:'MOVE', arg:'STOP'});
                } else {
                    // It was a short tap, send step command
                    send({cmd:'STEP', arg:b.dataset.arg});
                }
                isHolding = false;
            };
            
            b.addEventListener('touchstart', start, { passive: false });
            b.addEventListener('touchend', end, { passive: false });
            b.addEventListener('touchcancel', end, { passive: false });
            b.addEventListener('mousedown', start);
            b.addEventListener('mouseup', end);
            b.addEventListener('mouseleave', end);
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


const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Glitch — Robot Dashboard</title>
    <meta name="description" content="Real-time sensor data and camera feed dashboard for the Glitch mecanum robot">
    <link rel="preconnect" href="https://fonts.googleapis.com">
    <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
    <link href="https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700;800&family=JetBrains+Mono:wght@400;500&display=swap" rel="stylesheet">
    <style>
        /* ═══════════════════════════════════════════
           Design System
           ═══════════════════════════════════════════ */
        :root {
            --bg-primary: #0a0e17;
            --bg-secondary: #111827;
            --bg-card: rgba(17, 24, 39, 0.7);
            --bg-card-hover: rgba(22, 31, 48, 0.85);
            --border-color: rgba(99, 102, 241, 0.15);
            --border-glow: rgba(99, 102, 241, 0.35);
            --text-primary: #f1f5f9;
            --text-secondary: #94a3b8;
            --text-muted: #64748b;
            --accent-indigo: #818cf8;
            --accent-blue: #60a5fa;
            --accent-cyan: #22d3ee;
            --accent-emerald: #34d399;
            --accent-amber: #fbbf24;
            --accent-rose: #fb7185;
            --accent-violet: #a78bfa;
            --status-ok: #34d399;
            --status-warn: #fbbf24;
            --status-error: #fb7185;
            --radius-sm: 8px;
            --radius-md: 12px;
            --radius-lg: 16px;
            --radius-xl: 20px;
            --shadow-card: 0 4px 24px rgba(0,0,0,0.3), 0 0 0 1px var(--border-color);
            --shadow-glow: 0 0 20px rgba(99,102,241,0.12);
            --font-sans: 'Inter', -apple-system, BlinkMacSystemFont, sans-serif;
            --font-mono: 'JetBrains Mono', 'Fira Code', monospace;
            --transition-fast: 150ms cubic-bezier(0.4, 0, 0.2, 1);
            --transition-smooth: 300ms cubic-bezier(0.4, 0, 0.2, 1);
        }

        *, *::before, *::after {
            box-sizing: border-box;
            margin: 0;
            padding: 0;
        }

        html {
            font-size: 14px;
            scroll-behavior: smooth;
        }

        body {
            font-family: var(--font-sans);
            background: var(--bg-primary);
            color: var(--text-primary);
            min-height: 100vh;
            overflow-x: hidden;
            line-height: 1.5;
        }

        /* Background gradient mesh */
        body::before {
            content: '';
            position: fixed;
            top: 0; left: 0;
            width: 100%; height: 100%;
            background:
                radial-gradient(ellipse at 15% 10%, rgba(99,102,241,0.08) 0%, transparent 50%),
                radial-gradient(ellipse at 85% 20%, rgba(34,211,238,0.06) 0%, transparent 50%),
                radial-gradient(ellipse at 50% 80%, rgba(167,139,250,0.05) 0%, transparent 50%);
            pointer-events: none;
            z-index: 0;
        }

        /* ═══════════════════════════════════════════
           Layout
           ═══════════════════════════════════════════ */
        .app-container {
            position: relative;
            z-index: 1;
            max-width: 1440px;
            margin: 0 auto;
            padding: 20px;
        }

        /* ═══════════════════════════════════════════
           Header
           ═══════════════════════════════════════════ */
        .header {
            display: flex;
            align-items: center;
            justify-content: space-between;
            padding: 16px 24px;
            margin-bottom: 20px;
            background: var(--bg-card);
            backdrop-filter: blur(16px);
            border-radius: var(--radius-lg);
            border: 1px solid var(--border-color);
            box-shadow: var(--shadow-card);
        }

        .header-left {
            display: flex;
            align-items: center;
            gap: 14px;
        }

        .logo {
            width: 42px;
            height: 42px;
            background: linear-gradient(135deg, var(--accent-indigo), var(--accent-cyan));
            border-radius: var(--radius-sm);
            display: flex;
            align-items: center;
            justify-content: center;
            font-weight: 800;
            font-size: 1.3rem;
            letter-spacing: -1px;
            box-shadow: 0 0 16px rgba(99,102,241,0.25);
        }

        .header-title {
            font-size: 1.4rem;
            font-weight: 700;
            background: linear-gradient(135deg, var(--text-primary), var(--accent-blue));
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            background-clip: text;
        }

        .header-subtitle {
            font-size: 0.78rem;
            color: var(--text-muted);
            font-weight: 400;
        }

        .header-right {
            display: flex;
            align-items: center;
            gap: 16px;
        }

        .connection-badge {
            display: flex;
            align-items: center;
            gap: 6px;
            padding: 6px 14px;
            border-radius: 20px;
            font-size: 0.75rem;
            font-weight: 500;
            border: 1px solid;
            transition: var(--transition-smooth);
        }

        .connection-badge.ok {
            color: var(--status-ok);
            border-color: rgba(52,211,153,0.25);
            background: rgba(52,211,153,0.08);
        }

        .connection-badge.error {
            color: var(--status-error);
            border-color: rgba(251,113,133,0.25);
            background: rgba(251,113,133,0.08);
        }

        .connection-dot {
            width: 7px;
            height: 7px;
            border-radius: 50%;
            animation: pulse-dot 2s ease-in-out infinite;
        }

        .connection-badge.ok .connection-dot { background: var(--status-ok); }
        .connection-badge.error .connection-dot { background: var(--status-error); animation: none; }

        @keyframes pulse-dot {
            0%, 100% { opacity: 1; transform: scale(1); }
            50% { opacity: 0.5; transform: scale(0.8); }
        }

        /* ═══════════════════════════════════════════
           Config Bar
           ═══════════════════════════════════════════ */
        .config-bar {
            display: flex;
            align-items: center;
            gap: 12px;
            padding: 12px 20px;
            margin-bottom: 20px;
            background: var(--bg-card);
            backdrop-filter: blur(16px);
            border-radius: var(--radius-md);
            border: 1px solid var(--border-color);
            flex-wrap: wrap;
        }

        .config-bar label {
            font-size: 0.78rem;
            color: var(--text-secondary);
            font-weight: 500;
        }

        .config-bar input[type="text"] {
            background: rgba(0,0,0,0.3);
            border: 1px solid var(--border-color);
            border-radius: var(--radius-sm);
            color: var(--text-primary);
            font-family: var(--font-mono);
            font-size: 0.8rem;
            padding: 6px 12px;
            width: 160px;
            outline: none;
            transition: var(--transition-fast);
        }

        .config-bar input[type="text"]:focus {
            border-color: var(--accent-indigo);
            box-shadow: 0 0 0 2px rgba(99,102,241,0.15);
        }

        .config-bar .separator {
            width: 1px;
            height: 24px;
            background: var(--border-color);
        }

        .btn {
            padding: 7px 18px;
            border-radius: var(--radius-sm);
            border: 1px solid var(--border-color);
            background: rgba(99,102,241,0.1);
            color: var(--accent-indigo);
            font-family: var(--font-sans);
            font-size: 0.78rem;
            font-weight: 600;
            cursor: pointer;
            transition: var(--transition-fast);
        }

        .btn:hover {
            background: rgba(99,102,241,0.2);
            border-color: var(--border-glow);
            box-shadow: var(--shadow-glow);
        }

        .btn-primary {
            background: linear-gradient(135deg, rgba(99,102,241,0.3), rgba(34,211,238,0.15));
            color: var(--accent-cyan);
            border-color: rgba(34,211,238,0.25);
        }

        .btn-primary:hover {
            background: linear-gradient(135deg, rgba(99,102,241,0.45), rgba(34,211,238,0.25));
        }

        /* ═══════════════════════════════════════════
           Grid Layout
           ═══════════════════════════════════════════ */
        .dashboard-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            grid-template-rows: auto;
            gap: 16px;
        }

        /* ═══════════════════════════════════════════
           Cards
           ═══════════════════════════════════════════ */
        .card {
            background: var(--bg-card);
            backdrop-filter: blur(16px);
            border-radius: var(--radius-lg);
            border: 1px solid var(--border-color);
            box-shadow: var(--shadow-card);
            overflow: hidden;
            transition: var(--transition-smooth);
        }

        .card:hover {
            border-color: var(--border-glow);
            box-shadow: var(--shadow-card), var(--shadow-glow);
        }

        .card-header {
            display: flex;
            align-items: center;
            justify-content: space-between;
            padding: 14px 20px 10px;
            border-bottom: 1px solid rgba(255,255,255,0.04);
        }

        .card-title {
            display: flex;
            align-items: center;
            gap: 8px;
            font-size: 0.85rem;
            font-weight: 600;
            color: var(--text-primary);
        }

        .card-title .icon {
            font-size: 1rem;
            opacity: 0.8;
        }

        .card-badge {
            font-size: 0.65rem;
            padding: 2px 8px;
            border-radius: 10px;
            font-weight: 600;
            font-family: var(--font-mono);
        }

        .card-badge.live {
            color: var(--status-ok);
            background: rgba(52,211,153,0.12);
            border: 1px solid rgba(52,211,153,0.2);
        }

        .card-badge.stale {
            color: var(--status-warn);
            background: rgba(251,191,36,0.12);
            border: 1px solid rgba(251,191,36,0.2);
        }

        .card-badge.offline {
            color: var(--status-error);
            background: rgba(251,113,133,0.12);
            border: 1px solid rgba(251,113,133,0.2);
        }

        .card-body {
            padding: 14px 20px 18px;
        }

        /* ═══════════════════════════════════════════
           Camera Stream Card
           ═══════════════════════════════════════════ */
        .card.camera-card {
            grid-column: 1 / 2;
            grid-row: 1 / 3;
        }

        .stream-container {
            position: relative;
            background: #000;
            border-radius: var(--radius-sm);
            overflow: hidden;
            aspect-ratio: 4 / 3;
        }

        .stream-container img {
            width: 100%;
            height: 100%;
            object-fit: contain;
            display: block;
        }

        .stream-overlay {
            position: absolute;
            top: 0; left: 0;
            width: 100%; height: 100%;
            display: flex;
            align-items: center;
            justify-content: center;
            background: rgba(0,0,0,0.7);
            transition: var(--transition-smooth);
        }

        .stream-overlay.hidden {
            opacity: 0;
            pointer-events: none;
        }

        .stream-overlay-text {
            color: var(--text-muted);
            font-size: 0.9rem;
            text-align: center;
        }

        .stream-overlay-text .icon {
            font-size: 2.5rem;
            display: block;
            margin-bottom: 8px;
        }

        .stream-stats {
            position: absolute;
            bottom: 8px;
            left: 8px;
            display: flex;
            gap: 6px;
        }

        .stream-stat-pill {
            background: rgba(0,0,0,0.65);
            backdrop-filter: blur(6px);
            padding: 3px 8px;
            border-radius: 6px;
            font-size: 0.65rem;
            font-family: var(--font-mono);
            color: var(--text-secondary);
            border: 1px solid rgba(255,255,255,0.08);
        }

        /* ═══════════════════════════════════════════
           QR Detection Card
           ═══════════════════════════════════════════ */
        .qr-info-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 10px;
        }

        .qr-field {
            background: rgba(0,0,0,0.2);
            border-radius: var(--radius-sm);
            padding: 10px 14px;
            border: 1px solid rgba(255,255,255,0.04);
        }

        .qr-field-label {
            font-size: 0.65rem;
            color: var(--text-muted);
            text-transform: uppercase;
            letter-spacing: 0.5px;
            font-weight: 600;
            margin-bottom: 4px;
        }

        .qr-field-value {
            font-family: var(--font-mono);
            font-size: 1.1rem;
            font-weight: 600;
            color: var(--text-primary);
        }

        .qr-field-value.color-red { color: var(--accent-rose); }
        .qr-field-value.color-green { color: var(--accent-emerald); }
        .qr-field-value.color-blue { color: var(--accent-blue); }
        .qr-field-value.color-unknown { color: var(--text-muted); }

        .qr-field.span-2 { grid-column: 1 / -1; }

        /* ═══════════════════════════════════════════
           Pose Card
           ═══════════════════════════════════════════ */
        .pose-grid {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 8px;
        }

        .pose-axis {
            text-align: center;
            padding: 12px 8px;
            background: rgba(0,0,0,0.2);
            border-radius: var(--radius-sm);
            border: 1px solid rgba(255,255,255,0.04);
        }

        .pose-axis-label {
            font-size: 0.65rem;
            color: var(--text-muted);
            text-transform: uppercase;
            letter-spacing: 0.5px;
            font-weight: 600;
        }

        .pose-axis-value {
            font-family: var(--font-mono);
            font-size: 1.3rem;
            font-weight: 700;
            margin-top: 4px;
        }

        .pose-axis-unit {
            font-size: 0.65rem;
            color: var(--text-muted);
            font-weight: 400;
        }

        .pose-axis.x .pose-axis-value { color: var(--accent-rose); }
        .pose-axis.y .pose-axis-value { color: var(--accent-emerald); }
        .pose-axis.z .pose-axis-value { color: var(--accent-blue); }
        .pose-axis.roll .pose-axis-value { color: var(--accent-amber); }
        .pose-axis.pitch .pose-axis-value { color: var(--accent-violet); }
        .pose-axis.yaw .pose-axis-value { color: var(--accent-cyan); }

        .pose-section-label {
            font-size: 0.7rem;
            color: var(--text-muted);
            font-weight: 500;
            margin-top: 12px;
            margin-bottom: 6px;
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }

        /* ═══════════════════════════════════════════
           Confidence Bar
           ═══════════════════════════════════════════ */
        .confidence-bar-wrap {
            margin-top: 12px;
        }

        .confidence-bar-header {
            display: flex;
            justify-content: space-between;
            font-size: 0.7rem;
            margin-bottom: 5px;
        }

        .confidence-bar-label {
            color: var(--text-muted);
            font-weight: 500;
        }

        .confidence-bar-value {
            font-family: var(--font-mono);
            font-weight: 600;
        }

        .confidence-bar-track {
            height: 6px;
            background: rgba(255,255,255,0.06);
            border-radius: 3px;
            overflow: hidden;
        }

        .confidence-bar-fill {
            height: 100%;
            border-radius: 3px;
            transition: width 0.5s ease, background 0.5s ease;
            background: var(--accent-indigo);
        }

        .confidence-bar-fill.high { background: linear-gradient(90deg, var(--accent-emerald), var(--accent-cyan)); }
        .confidence-bar-fill.mid { background: linear-gradient(90deg, var(--accent-amber), var(--accent-emerald)); }
        .confidence-bar-fill.low { background: linear-gradient(90deg, var(--accent-rose), var(--accent-amber)); }

        /* ═══════════════════════════════════════════
           Health Card
           ═══════════════════════════════════════════ */
        .health-grid {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 8px;
        }

        .health-item {
            display: flex;
            align-items: center;
            justify-content: space-between;
            padding: 8px 12px;
            background: rgba(0,0,0,0.2);
            border-radius: var(--radius-sm);
            border: 1px solid rgba(255,255,255,0.04);
        }

        .health-item-label {
            font-size: 0.72rem;
            color: var(--text-secondary);
            font-weight: 500;
        }

        .health-item-value {
            font-family: var(--font-mono);
            font-size: 0.78rem;
            font-weight: 600;
            color: var(--text-primary);
        }

        .health-item.span-2 { grid-column: 1 / -1; }

        /* ═══════════════════════════════════════════
           Log Card
           ═══════════════════════════════════════════ */
        .card.log-card {
            grid-column: 1 / -1;
        }

        .log-container {
            max-height: 180px;
            overflow-y: auto;
            font-family: var(--font-mono);
            font-size: 0.72rem;
            line-height: 1.7;
            color: var(--text-secondary);
            background: rgba(0,0,0,0.2);
            border-radius: var(--radius-sm);
            padding: 10px 14px;
            border: 1px solid rgba(255,255,255,0.04);
        }

        .log-container::-webkit-scrollbar {
            width: 5px;
        }

        .log-container::-webkit-scrollbar-track {
            background: transparent;
        }

        .log-container::-webkit-scrollbar-thumb {
            background: rgba(255,255,255,0.1);
            border-radius: 3px;
        }

        .log-entry {
            white-space: nowrap;
        }

        .log-ts {
            color: var(--text-muted);
        }

        .log-tag {
            font-weight: 600;
        }

        .log-tag.cam { color: var(--accent-cyan); }
        .log-tag.qr { color: var(--accent-emerald); }
        .log-tag.sys { color: var(--accent-amber); }
        .log-tag.err { color: var(--accent-rose); }

        /* ═══════════════════════════════════════════
           Yaw Indicator
           ═══════════════════════════════════════════ */
        .yaw-indicator {
            position: relative;
            height: 40px;
            background: rgba(0,0,0,0.25);
            border-radius: var(--radius-sm);
            margin-top: 10px;
            overflow: hidden;
            border: 1px solid rgba(255,255,255,0.06);
        }

        .yaw-center-line {
            position: absolute;
            left: 50%;
            top: 0;
            width: 2px;
            height: 100%;
            background: rgba(255,255,255,0.15);
            transform: translateX(-50%);
        }

        .yaw-marker {
            position: absolute;
            top: 50%;
            width: 12px;
            height: 12px;
            background: var(--accent-cyan);
            border-radius: 50%;
            transform: translate(-50%, -50%);
            transition: left 0.3s ease;
            box-shadow: 0 0 8px rgba(34,211,238,0.5);
        }

        .yaw-label-left, .yaw-label-right {
            position: absolute;
            top: 50%;
            transform: translateY(-50%);
            font-size: 0.6rem;
            color: var(--text-muted);
            font-family: var(--font-mono);
        }
        .yaw-label-left { left: 8px; }
        .yaw-label-right { right: 8px; }
        .yaw-label-center {
            position: absolute;
            top: 2px;
            left: 50%;
            transform: translateX(-50%);
            font-size: 0.55rem;
            color: var(--text-muted);
            font-family: var(--font-mono);
        }

        /* ═══════════════════════════════════════════
           Motor Speed Gauge
           ═══════════════════════════════════════════ */
        .motor-speed-display {
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 16px;
            padding: 10px;
        }

        .speed-ring {
            position: relative;
            width: 100px;
            height: 100px;
        }

        .speed-ring svg {
            transform: rotate(-90deg);
        }

        .speed-ring-track {
            fill: none;
            stroke: rgba(255,255,255,0.06);
            stroke-width: 6;
        }

        .speed-ring-fill {
            fill: none;
            stroke: var(--accent-indigo);
            stroke-width: 6;
            stroke-linecap: round;
            transition: stroke-dashoffset 0.5s ease, stroke 0.3s ease;
        }

        .speed-ring-text {
            position: absolute;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            text-align: center;
        }

        .speed-ring-value {
            font-family: var(--font-mono);
            font-size: 1.5rem;
            font-weight: 700;
            color: var(--text-primary);
            line-height: 1;
        }

        .speed-ring-unit {
            font-size: 0.6rem;
            color: var(--text-muted);
            font-weight: 500;
        }

        /* ═══════════════════════════════════════════
           Responsive
           ═══════════════════════════════════════════ */
        @media (max-width: 900px) {
            .dashboard-grid {
                grid-template-columns: 1fr;
            }
            .card.camera-card {
                grid-column: 1;
                grid-row: auto;
            }
            .card.log-card {
                grid-column: 1;
            }
            .config-bar {
                flex-direction: column;
                align-items: flex-start;
            }
            .config-bar .separator { display: none; }
        }
    </style>
</head>
<body>
    <div class="app-container">
        <!-- ════════ Header ════════ -->
        <header class="header">
            <div class="header-left">
                <div class="logo">G</div>
                <div>
                    <div class="header-title">Glitch Dashboard</div>
                    <div class="header-subtitle">Mecanum Robot — Real-Time Data Display</div>
                </div>
            </div>
            <div class="header-right">
                <div id="badge-camera" class="connection-badge error">
                    <span class="connection-dot"></span>
                    <span>Camera</span>
                </div>
                <div id="badge-base" class="connection-badge error">
                    <span class="connection-dot"></span>
                    <span>Base WS</span>
                </div>
            </div>
        </header>

        <!-- ════════ Config Bar ════════ -->
        <div class="config-bar">
            <label for="input-cam-ip">Camera IP</label>
            <input type="text" id="input-cam-ip" value="192.168.5.100" placeholder="192.168.5.x">
            <div class="separator"></div>
            <label for="input-blynk-ip">Blynk Server</label>
            <input type="text" id="input-blynk-ip" value="192.168.5.1" placeholder="192.168.5.1">
            <label for="input-blynk-port">Port</label>
            <input type="text" id="input-blynk-port" value="8080" placeholder="8080" style="width:60px;">
            <label for="input-blynk-token">Auth Token</label>
            <input type="text" id="input-blynk-token" value="Kspg0_T5ov2BDlZ3-HMLCJoOoWtlRrqV" placeholder="auth_token">
            <div class="separator"></div>
            <button class="btn btn-primary" id="btn-connect" onclick="connectAll()">▶ Connect</button>
            <button class="btn" id="btn-disconnect" onclick="disconnectAll()">⏹ Stop</button>
        </div>

        <!-- ════════ Dashboard Grid ════════ -->
        <div class="dashboard-grid">

            <!-- Camera Stream -->
            <div class="card camera-card">
                <div class="card-header">
                    <div class="card-title"><span class="icon">📷</span> Camera Stream</div>
                    <span id="stream-badge" class="card-badge offline">OFFLINE</span>
                </div>
                <div class="card-body">
                    <div class="stream-container">
                        <img id="stream-img" src="" alt="Camera Stream">
                        <div id="stream-overlay" class="stream-overlay">
                            <div class="stream-overlay-text">
                                <span class="icon">📡</span>
                                Press <strong>Connect</strong> to start stream
                            </div>
                        </div>
                        <div class="stream-stats">
                            <span id="stream-fps" class="stream-stat-pill">— FPS</span>
                            <span id="stream-proc" class="stream-stat-pill">— ms</span>
                        </div>
                    </div>

                    <!-- Yaw Indicator under stream -->
                    <div style="margin-top:12px;">
                        <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:4px;">
                            <span style="font-size:0.7rem; color:var(--text-muted); font-weight:500;">QR Yaw Offset</span>
                            <span id="yaw-value-text" style="font-family:var(--font-mono); font-size:0.75rem; color:var(--accent-cyan); font-weight:600;">—°</span>
                        </div>
                        <div class="yaw-indicator">
                            <div class="yaw-center-line"></div>
                            <div id="yaw-marker" class="yaw-marker" style="left:50%;"></div>
                            <span class="yaw-label-left">◄ LEFT</span>
                            <span class="yaw-label-right">RIGHT ►</span>
                            <span class="yaw-label-center">0°</span>
                        </div>
                    </div>
                </div>
            </div>

            <!-- QR Detection -->
            <div class="card">
                <div class="card-header">
                    <div class="card-title"><span class="icon">🔍</span> QR Detection</div>
                    <span id="qr-badge" class="card-badge offline">NO DATA</span>
                </div>
                <div class="card-body">
                    <div class="qr-info-grid">
                        <div class="qr-field span-2">
                            <div class="qr-field-label">Detected Text</div>
                            <div id="qr-text" class="qr-field-value color-unknown">—</div>
                        </div>
                        <div class="qr-field">
                            <div class="qr-field-label">Decoded</div>
                            <div id="qr-decoded" class="qr-field-value">—</div>
                        </div>
                        <div class="qr-field">
                            <div class="qr-field-label">Estimated</div>
                            <div id="qr-estimated" class="qr-field-value">—</div>
                        </div>
                        <div class="qr-field">
                            <div class="qr-field-label">Age</div>
                            <div id="qr-age" class="qr-field-value">— ms</div>
                        </div>
                        <div class="qr-field">
                            <div class="qr-field-label">Frame</div>
                            <div id="qr-frame" class="qr-field-value">#—</div>
                        </div>
                    </div>

                    <!-- Confidence Bar -->
                    <div class="confidence-bar-wrap">
                        <div class="confidence-bar-header">
                            <span class="confidence-bar-label">Confidence</span>
                            <span id="conf-value" class="confidence-bar-value" style="color:var(--text-muted);">—</span>
                        </div>
                        <div class="confidence-bar-track">
                            <div id="conf-fill" class="confidence-bar-fill" style="width:0%;"></div>
                        </div>
                    </div>
                </div>
            </div>

            <!-- Pose Data -->
            <div class="card">
                <div class="card-header">
                    <div class="card-title"><span class="icon">🎯</span> Pose Estimation</div>
                    <span id="pose-badge" class="card-badge offline">NO POSE</span>
                </div>
                <div class="card-body">
                    <div class="pose-section-label">Translation (mm)</div>
                    <div class="pose-grid">
                        <div class="pose-axis x">
                            <div class="pose-axis-label">X</div>
                            <div id="pose-tx" class="pose-axis-value">—</div>
                            <div class="pose-axis-unit">mm</div>
                        </div>
                        <div class="pose-axis y">
                            <div class="pose-axis-label">Y</div>
                            <div id="pose-ty" class="pose-axis-value">—</div>
                            <div class="pose-axis-unit">mm</div>
                        </div>
                        <div class="pose-axis z">
                            <div class="pose-axis-label">Z</div>
                            <div id="pose-tz" class="pose-axis-value">—</div>
                            <div class="pose-axis-unit">mm</div>
                        </div>
                    </div>
                    <div class="pose-section-label">Rotation (deg)</div>
                    <div class="pose-grid">
                        <div class="pose-axis roll">
                            <div class="pose-axis-label">Roll</div>
                            <div id="pose-roll" class="pose-axis-value">—</div>
                            <div class="pose-axis-unit">°</div>
                        </div>
                        <div class="pose-axis pitch">
                            <div class="pose-axis-label">Pitch</div>
                            <div id="pose-pitch" class="pose-axis-value">—</div>
                            <div class="pose-axis-unit">°</div>
                        </div>
                        <div class="pose-axis yaw">
                            <div class="pose-axis-label">Yaw</div>
                            <div id="pose-yaw" class="pose-axis-value">—</div>
                            <div class="pose-axis-unit">°</div>
                        </div>
                    </div>
                </div>
            </div>

            <!-- System Health -->
            <div class="card">
                <div class="card-header">
                    <div class="card-title"><span class="icon">💚</span> System Health</div>
                    <span id="health-badge" class="card-badge offline">OFFLINE</span>
                </div>
                <div class="card-body">
                    <div class="health-grid">
                        <div class="health-item">
                            <span class="health-item-label">Camera</span>
                            <span id="health-cam" class="health-item-value" style="color:var(--text-muted);">—</span>
                        </div>
                        <div class="health-item">
                            <span class="health-item-label">QR FPS</span>
                            <span id="health-fps" class="health-item-value">—</span>
                        </div>
                        <div class="health-item">
                            <span class="health-item-label">Free Heap</span>
                            <span id="health-heap" class="health-item-value">—</span>
                        </div>
                        <div class="health-item">
                            <span class="health-item-label">PSRAM Free</span>
                            <span id="health-psram" class="health-item-value">—</span>
                        </div>
                        <div class="health-item">
                            <span class="health-item-label">QR Proc</span>
                            <span id="health-proc" class="health-item-value">—</span>
                        </div>
                        <div class="health-item">
                            <span class="health-item-label">Tracker</span>
                            <span id="health-track" class="health-item-value" style="color:var(--text-muted);">—</span>
                        </div>
                        <div class="health-item">
                            <span class="health-item-label">Motor Speed</span>
                            <span id="health-motor" class="health-item-value">—</span>
                        </div>
                        <div class="health-item">
                            <span class="health-item-label">WiFi</span>
                            <span id="health-wifi" class="health-item-value" style="color:var(--text-muted);">—</span>
                        </div>
                    </div>

                    <!-- Motor Speed Ring Gauge -->
                    <div class="motor-speed-display">
                        <div class="speed-ring">
                            <svg width="100" height="100" viewBox="0 0 100 100">
                                <circle class="speed-ring-track" cx="50" cy="50" r="42"></circle>
                                <circle id="speed-ring-fill" class="speed-ring-fill" cx="50" cy="50" r="42"
                                    stroke-dasharray="263.9" stroke-dashoffset="263.9"></circle>
                            </svg>
                            <div class="speed-ring-text">
                                <div id="speed-ring-value" class="speed-ring-value">0</div>
                                <div class="speed-ring-unit">speed</div>
                            </div>
                        </div>
                    </div>
                </div>
            </div>

            <!-- Platform Detection -->
            <div class="card">
                <div class="card-header">
                    <div class="card-title"><span class="icon">⬛</span> Platform Detection</div>
                    <span id="plat-badge" class="card-badge offline">NO DATA</span>
                </div>
                <div class="card-body">
                    <div class="qr-info-grid">
                        <div class="qr-field">
                            <div class="qr-field-label">Detected</div>
                            <div id="plat-detected" class="qr-field-value">—</div>
                        </div>
                        <div class="qr-field">
                            <div class="qr-field-label">Distance</div>
                            <div id="plat-dist" class="qr-field-value">— mm</div>
                        </div>
                        <div class="qr-field">
                            <div class="qr-field-label">Center (X,Y)</div>
                            <div id="plat-center" class="qr-field-value">—, —</div>
                        </div>
                        <div class="qr-field">
                            <div class="qr-field-label">Size (W×H)</div>
                            <div id="plat-size" class="qr-field-value">— × —</div>
                        </div>
                        <div class="qr-field">
                            <div class="qr-field-label">Angle</div>
                            <div id="plat-angle" class="qr-field-value">—°</div>
                        </div>
                        <div class="qr-field">
                            <div class="qr-field-label">Process Time</div>
                            <div id="plat-proc" class="qr-field-value">— ms</div>
                        </div>
                    </div>

                    <!-- Platform Confidence Bar -->
                    <div class="confidence-bar-wrap">
                        <div class="confidence-bar-header">
                            <span class="confidence-bar-label">Confidence</span>
                            <span id="plat-conf-value" class="confidence-bar-value" style="color:var(--text-muted);">—</span>
                        </div>
                        <div class="confidence-bar-track">
                            <div id="plat-conf-fill" class="confidence-bar-fill" style="width:0%;"></div>
                        </div>
                    </div>
                </div>
            </div>

            <!-- Event Log -->
            <div class="card log-card">
                <div class="card-header">
                    <div class="card-title"><span class="icon">📋</span> Event Log</div>
                    <button class="btn" onclick="clearLog()" style="padding:3px 10px; font-size:0.68rem;">Clear</button>
                </div>
                <div class="card-body">
                    <div id="log-container" class="log-container">
                        <div class="log-entry"><span class="log-ts">[--:--:--]</span> <span class="log-tag sys">SYS</span> Dashboard loaded. Enter IPs and press Connect.</div>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <script>
    /* ═══════════════════════════════════════════
       State
       ═══════════════════════════════════════════ */
    let camIP = '';
    let baseIP = '';
    let polling = false;
    let dataInterval = null;
    let statusInterval = null;
    let platformInterval = null;
    let cameraConnected = false;
    let baseConnected = false;
    let baseWs = null;
    let lastDataTime = 0;

    /* ═══════════════════════════════════════════
       Logging
       ═══════════════════════════════════════════ */
    function log(tag, msg) {
        const container = document.getElementById('log-container');
        const now = new Date();
        const ts = now.toTimeString().slice(0, 8);
        const entry = document.createElement('div');
        entry.className = 'log-entry';
        entry.innerHTML = `<span class="log-ts">[${ts}]</span> <span class="log-tag ${tag}">${tag.toUpperCase()}</span> ${msg}`;
        container.appendChild(entry);
        container.scrollTop = container.scrollHeight;
        // Keep last 200 entries
        while (container.children.length > 200) {
            container.removeChild(container.firstChild);
        }
    }

    function clearLog() {
        document.getElementById('log-container').innerHTML = '';
        log('sys', 'Log cleared.');
    }

    /* ═══════════════════════════════════════════
       Connection
       ═══════════════════════════════════════════ */
    function connectAll() {
        camIP = document.getElementById('input-cam-ip').value.trim();
        baseIP = document.getElementById('input-base-ip').value.trim();

        if (!camIP) {
            log('err', 'Camera IP is required.');
            return;
        }

        disconnectAll();
        polling = true;
        log('sys', `Connecting to camera at ${camIP}...`);

        // Start MJPEG stream
        const streamImg = document.getElementById('stream-img');
        streamImg.src = `http://${camIP}:81/stream`;
        streamImg.onload = function() {
            document.getElementById('stream-overlay').classList.add('hidden');
            setBadge('stream-badge', 'LIVE', 'live');
            cameraConnected = true;
            updateConnectionBadge('badge-camera', true);
            log('cam', 'MJPEG stream connected.');
        };
        streamImg.onerror = function() {
            document.getElementById('stream-overlay').classList.remove('hidden');
            setBadge('stream-badge', 'ERROR', 'offline');
            cameraConnected = false;
            updateConnectionBadge('badge-camera', false);
            log('err', 'Camera stream failed to connect. Check IP and network.');
        };

        // Start polling /data every 500ms
        dataInterval = setInterval(fetchData, 500);
        fetchData();

        // Start polling /status every 5s
        statusInterval = setInterval(fetchStatus, 5000);
        fetchStatus();

        // Start polling /platform every 1s
        platformInterval = setInterval(fetchPlatform, 1000);
        fetchPlatform();

        // Connect to Base WebSocket
        if (baseIP) {
            log('sys', `Connecting to Base WS at ${baseIP}...`);
            baseWs = new WebSocket(`ws://${baseIP}/ws`);
            baseWs.onopen = () => {
                baseConnected = true;
                updateConnectionBadge('badge-base', true);
                log('sys', 'Base WS connected.');
            };
            baseWs.onclose = () => {
                baseConnected = false;
                updateConnectionBadge('badge-base', false);
                log('err', 'Base WS disconnected.');
            };
            baseWs.onmessage = (e) => {
                try {
                    const d = JSON.parse(e.data);
                    if (d.motor_speed !== undefined) {
                        const speed = d.motor_speed;
                        document.getElementById('health-motor').textContent = speed;
                        
                        const maxSpeed = 100;
                        const pct = Math.min(speed / maxSpeed, 1);
                        const circumference = 2 * Math.PI * 42;
                        const offset = circumference * (1 - pct);
                        const ringFill = document.getElementById('speed-ring-fill');
                        ringFill.style.strokeDashoffset = offset;
                        ringFill.style.stroke = speed > 70 ? 'var(--accent-rose)' : speed > 40 ? 'var(--accent-amber)' : 'var(--accent-emerald)';
                        document.getElementById('speed-ring-value').textContent = speed;
                    }
                } catch(ex){}
            };
        }
    }

    function disconnectAll() {
        polling = false;
        if (dataInterval) { clearInterval(dataInterval); dataInterval = null; }
        if (statusInterval) { clearInterval(statusInterval); statusInterval = null; }
        if (platformInterval) { clearInterval(platformInterval); platformInterval = null; }
        if (baseWs) { baseWs.close(); baseWs = null; }

        document.getElementById('stream-img').src = '';
        document.getElementById('stream-overlay').classList.remove('hidden');
        setBadge('stream-badge', 'OFFLINE', 'offline');
        cameraConnected = false;
        baseConnected = false;
        updateConnectionBadge('badge-camera', false);
        updateConnectionBadge('badge-base', false);
        log('sys', 'Disconnected.');
    }

    /* ═══════════════════════════════════════════
       Camera /data polling
       ═══════════════════════════════════════════ */
    async function fetchData() {
        if (!polling || !camIP) return;
        try {
            const resp = await fetch(`http://${camIP}/data`, { signal: AbortSignal.timeout(2000) });
            const d = await resp.json();
            lastDataTime = Date.now();

            cameraConnected = true;
            updateConnectionBadge('badge-camera', true);

            // Frame info
            document.getElementById('qr-frame').textContent = `#${d.frame_id}`;
            document.getElementById('stream-fps').textContent = `${d.qr_fps?.toFixed(1) || '—'} FPS`;
            document.getElementById('stream-proc').textContent = `${d.processing_ms || '—'} ms`;

            // QR detections
            if (d.qr_codes && d.qr_codes.length > 0) {
                const qr = d.qr_codes[0]; // primary detection (tracker output)
                setBadge('qr-badge', qr.decoded ? 'DECODED' : 'DETECTED', qr.decoded ? 'live' : 'stale');

                // Text + color
                const textEl = document.getElementById('qr-text');
                textEl.textContent = qr.text || '—';
                textEl.className = 'qr-field-value ' + getColorClass(qr.text);

                document.getElementById('qr-decoded').textContent = qr.decoded ? '✔ Yes' : '✘ No';
                document.getElementById('qr-decoded').style.color = qr.decoded ? 'var(--status-ok)' : 'var(--status-warn)';

                document.getElementById('qr-estimated').textContent = qr.estimated ? '⚠ Yes' : '✔ Fresh';
                document.getElementById('qr-estimated').style.color = qr.estimated ? 'var(--status-warn)' : 'var(--status-ok)';

                document.getElementById('qr-age').textContent = `${qr.age_ms} ms`;
                
                // Update confidence bar
                const conf = qr.confidence * 100;
                const confFill = document.getElementById('conf-fill');
                confFill.style.width = `${Math.min(100, Math.max(0, conf))}%`;
                document.getElementById('conf-value').textContent = `${conf.toFixed(1)}%`;
                
                confFill.className = 'confidence-bar-fill';
                if (conf > 80) confFill.classList.add('high');
                else if (conf > 40) confFill.classList.add('mid');
                else confFill.classList.add('low');

                // Update pose
                document.getElementById('pose-tx').textContent = qr.pose.tx.toFixed(1);
                document.getElementById('pose-ty').textContent = qr.pose.ty.toFixed(1);
                document.getElementById('pose-tz').textContent = qr.pose.tz.toFixed(1);
                document.getElementById('pose-roll').textContent = qr.pose.roll.toFixed(1);
                document.getElementById('pose-pitch').textContent = qr.pose.pitch.toFixed(1);
                document.getElementById('pose-yaw').textContent = qr.pose.yaw.toFixed(1);
                document.getElementById('pose-badge').textContent = 'VALID';
                document.getElementById('pose-badge').className = 'card-badge live';

                // Update Yaw visual indicator
                const yaw = qr.pose.yaw;
                document.getElementById('yaw-value-text').textContent = `${yaw > 0 ? '+' : ''}${yaw.toFixed(1)}°`;
                // Map -30°..30° to 0..100%
                let markerPct = 50 + (yaw / 30.0) * 50;
                markerPct = Math.max(5, Math.min(95, markerPct));
                document.getElementById('yaw-marker').style.left = `${markerPct}%`;

            } else {
                setBadge('qr-badge', 'NO QR', 'offline');
                document.getElementById('qr-text').textContent = '—';
                document.getElementById('qr-text').className = 'qr-field-value color-unknown';
                document.getElementById('qr-decoded').textContent = '—';
                document.getElementById('qr-decoded').style.color = '';
                document.getElementById('qr-estimated').textContent = '—';
                document.getElementById('qr-estimated').style.color = '';
                document.getElementById('qr-age').textContent = '— ms';
                
                document.getElementById('conf-fill').style.width = '0%';
                document.getElementById('conf-value').textContent = '—';

                document.getElementById('pose-tx').textContent = '—';
                document.getElementById('pose-ty').textContent = '—';
                document.getElementById('pose-tz').textContent = '—';
                document.getElementById('pose-roll').textContent = '—';
                document.getElementById('pose-pitch').textContent = '—';
                document.getElementById('pose-yaw').textContent = '—';
                document.getElementById('pose-badge').textContent = 'NO POSE';
                document.getElementById('pose-badge').className = 'card-badge offline';
                
                document.getElementById('yaw-value-text').textContent = '—°';
                document.getElementById('yaw-marker').style.left = '50%';
            }

        } catch (err) {
            console.error('Data fetch error:', err);
            // If we miss data for > 3s, mark offline
            if (Date.now() - lastDataTime > 3000) {
                cameraConnected = false;
                updateConnectionBadge('badge-camera', false);
                setBadge('qr-badge', 'OFFLINE', 'offline');
                setBadge('pose-badge', 'OFFLINE', 'offline');
            }
        }
    }

    /* ═══════════════════════════════════════════
       Platform /platform polling
       ═══════════════════════════════════════════ */
    async function fetchPlatform() {
        if (!polling || !camIP) return;
        try {
            const resp = await fetch(`http://${camIP}/platform`, { signal: AbortSignal.timeout(2000) });
            const p = await resp.json();

            if (p.detected) {
                setBadge('plat-badge', 'DETECTED', 'live');
                document.getElementById('plat-detected').textContent = '✔ Yes';
                document.getElementById('plat-detected').style.color = 'var(--status-ok)';
            } else {
                setBadge('plat-badge', 'NOT DETECTED', 'offline');
                document.getElementById('plat-detected').textContent = '✘ No';
                document.getElementById('plat-detected').style.color = 'var(--status-error)';
            }

            document.getElementById('plat-dist').textContent = p.detected ? `${p.distance_mm.toFixed(0)} mm` : '—';
            document.getElementById('plat-center').textContent = p.detected ? `${p.center_x.toFixed(0)}, ${p.center_y.toFixed(0)}` : '—, —';
            document.getElementById('plat-size').textContent = p.detected ? `${p.width_px.toFixed(0)} × ${p.height_px.toFixed(0)}` : '— × —';
            document.getElementById('plat-angle').textContent = p.detected ? `${p.angle_deg.toFixed(1)}°` : '—°';
            document.getElementById('plat-proc').textContent = `${p.processing_ms} ms`;

            // Confidence bar
            const conf = p.confidence * 100;
            const confFill = document.getElementById('plat-conf-fill');
            confFill.style.width = p.detected ? `${Math.min(100, Math.max(0, conf))}%` : '0%';
            document.getElementById('plat-conf-value').textContent = p.detected ? `${conf.toFixed(1)}%` : '—';
            
            confFill.className = 'confidence-bar-fill';
            if (conf > 80) confFill.classList.add('high');
            else if (conf > 40) confFill.classList.add('mid');
            else confFill.classList.add('low');

        } catch (err) {
            console.error('Platform fetch error:', err);
            setBadge('plat-badge', 'ERROR', 'offline');
        }
    }

    /* ═══════════════════════════════════════════
       Camera /status polling
       ═══════════════════════════════════════════ */
    async function fetchStatus() {
        if (!polling || !camIP) return;
        try {
            const resp = await fetch(`http://${camIP}/status`, { signal: AbortSignal.timeout(3000) });
            const s = await resp.json();

            setBadge('health-badge', 'ONLINE', 'live');

            document.getElementById('health-cam').textContent = s.camera || '—';
            document.getElementById('health-cam').style.color = s.camera === 'OK' ? 'var(--status-ok)' : 'var(--status-error)';

            document.getElementById('health-fps').textContent = `${s.qr_fps?.toFixed(1) || '—'}`;
            document.getElementById('health-heap').textContent = formatBytes(s.free_heap);
            document.getElementById('health-psram').textContent = formatBytes(s.psram_free);
            document.getElementById('health-proc').textContent = `${s.qr_processing_ms || '—'} ms`;

            document.getElementById('health-track').textContent = s.track_active ? `Active (${(s.track_conf * 100).toFixed(0)}%)` : 'Inactive';
            document.getElementById('health-track').style.color = s.track_active ? 'var(--status-ok)' : 'var(--text-muted)';

            document.getElementById('health-wifi').textContent = 'Connected';
            document.getElementById('health-wifi').style.color = 'var(--status-ok)';

        } catch (e) {
            setBadge('health-badge', 'OFFLINE', 'offline');
            document.getElementById('health-wifi').textContent = 'Down';
            document.getElementById('health-wifi').style.color = 'var(--status-error)';
        }
    }

    /* ═══════════════════════════════════════════
       (Blynk API removed)
       ═══════════════════════════════════════════ */

    /* ═══════════════════════════════════════════
       Helpers
       ═══════════════════════════════════════════ */
    function setBadge(id, text, cls) {
        const el = document.getElementById(id);
        el.textContent = text;
        el.className = 'card-badge ' + cls;
    }

    function updateConnectionBadge(id, connected) {
        const el = document.getElementById(id);
        el.className = 'connection-badge ' + (connected ? 'ok' : 'error');
    }

    function getColorClass(text) {
        if (!text) return 'color-unknown';
        const t = text.toUpperCase();
        if (t.includes('RED') || t === 'R') return 'color-red';
        if (t.includes('GREEN') || t === 'G') return 'color-green';
        if (t.includes('BLUE') || t === 'B') return 'color-blue';
        return 'color-unknown';
    }

    function formatBytes(bytes) {
        if (!bytes && bytes !== 0) return '—';
        if (bytes > 1048576) return `${(bytes / 1048576).toFixed(1)} MB`;
        if (bytes > 1024) return `${(bytes / 1024).toFixed(0)} KB`;
        return `${bytes} B`;
    }

    /* ═══════════════════════════════════════════
       Keyboard shortcut
       ═══════════════════════════════════════════ */
    document.addEventListener('keydown', (e) => {
        if (e.key === 'Enter' && (e.ctrlKey || e.metaKey)) {
            connectAll();
        }
        if (e.key === 'Escape') {
            disconnectAll();
        }
    });

    /* ═══════════════════════════════════════════
       Auto-save config to localStorage
       ═══════════════════════════════════════════ */
    function saveConfig() {
        const cfg = {
            camIP: document.getElementById('input-cam-ip').value,
            baseIP: document.getElementById('input-base-ip').value,
        };
        localStorage.setItem('glitch-dashboard-config', JSON.stringify(cfg));
    }

    function loadConfig() {
        try {
            const cfg = JSON.parse(localStorage.getItem('glitch-dashboard-config'));
            if (cfg) {
                if (cfg.camIP) document.getElementById('input-cam-ip').value = cfg.camIP;
                if (cfg.baseIP) document.getElementById('input-base-ip').value = cfg.baseIP;
            }
        } catch(e) {}
    }

    // Save on any input change
    document.querySelectorAll('.config-bar input').forEach(el => {
        el.addEventListener('input', saveConfig);
    });

    // Load on startup
    loadConfig();
    </script>
</body>
</html>

)rawliteral";


