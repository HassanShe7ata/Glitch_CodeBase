// =========================== ARM ESP32 (WebSocket) ===========================
// Receives commands via WebSocket (TCP) from Base
// Falls back to local AP+STA when base is offline.
// Callback only enqueues; loop() dispatches — never block in callback.
// WiFi reconnect is throttled; I2C errors logged.

#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <WebServer.h>
#include <WebSocketsClient.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_wifi.h>
#include <math.h>

// ================= TYPES (must be before all functions for PlatformIO ino2cpp)
// =================
struct JointAngles {
  int t1, t2, t3, t4;
  bool reachable;
};

// ================= COMMAND QUEUE (ring buffer) =================
// SPSC: writer = WiFi/WebSocket task, reader = loop().
// Uses portMUX spinlock for cross-core memory visibility on ESP32 dual-core.
#define CMD_Q_DEPTH 8
static char cmdQueue[CMD_Q_DEPTH][10];
static uint8_t cmdQHead = 0; // written by WiFi task (core 0)
static uint8_t cmdQTail = 0; // written by loop (core 1)
static portMUX_TYPE cmdMux = portMUX_INITIALIZER_UNLOCKED;

static bool enqueueCmd(const char *cmd) {
  portENTER_CRITICAL_ISR(&cmdMux);
  uint8_t next = (cmdQHead + 1) % CMD_Q_DEPTH;
  if (next == cmdQTail) {
    portEXIT_CRITICAL_ISR(&cmdMux);
    return false;
  }
  strncpy(cmdQueue[cmdQHead], cmd, 9);
  cmdQueue[cmdQHead][9] = '\0';
  cmdQHead = next;
  portEXIT_CRITICAL_ISR(&cmdMux);
  return true;
}

static bool dequeueCmd(char *out) {
  portENTER_CRITICAL(&cmdMux);
  if (cmdQHead == cmdQTail) {
    portEXIT_CRITICAL(&cmdMux);
    return false;
  }
  strncpy(out, cmdQueue[cmdQTail], 9);
  out[9] = '\0';
  cmdQTail = (cmdQTail + 1) % CMD_Q_DEPTH;
  portEXIT_CRITICAL(&cmdMux);
  return true;
}

// ================= WEBSOCKET (PRIMARY) =================
typedef struct {
  char command[10];
} ArmCommand;

typedef struct __attribute__((packed)) {
  uint8_t type; // 0=status
  uint8_t busy; // 1=busy, 0=idle
  uint8_t pad[2];
} ArmStatus;

// Camera pose data forwarded by base via WebSocket
struct __attribute__((packed)) CameraPoseData {
  uint8_t type; // 1 = camera pose packet
  uint8_t pose_valid;
  uint8_t color; // 0=unknown, 1=R, 2=G, 3=B
  uint8_t estimated;
  float tx_mm;
  float ty_mm;
  float tz_mm;
  float yaw_deg;
  float confidence;
};

static volatile bool cameraPoseReady = false;
static CameraPoseData incomingCameraPose;

// ================= BASE WEBSOCKET =================
static const char *BASE_WS_HOST = "192.168.4.1";
static const uint16_t BASE_WS_PORT = 8080;
static WebSocketsClient wsClient;

// WebSocket event handler — receives commands and camera poses from base
static void onWsEvent(WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_CONNECTED) {
    Serial.println("[WS] Connected to base");
  } else if (type == WStype_DISCONNECTED) {
    Serial.println("[WS] Disconnected from base");
  } else if (type == WStype_BIN && length >= 4) {
    // Camera pose packet (24 bytes, type=1)
    if (length == sizeof(CameraPoseData)) {
      CameraPoseData pose;
      memcpy(&pose, payload, sizeof(pose));
      if (pose.type == 1 && pose.pose_valid) {
        memcpy((void *)&incomingCameraPose, &pose, sizeof(pose));
        cameraPoseReady = true;
        Serial.printf("[WS-ARM] Camera pose: color=%d conf=%.2f "
                      "qr=(%.0f,%.0f,%.0f)\n",
                      pose.color, pose.confidence, pose.tx_mm, pose.ty_mm,
                      pose.tz_mm);
      }
      return;
    }
    // Arm command (10 bytes)
    if (length >= sizeof(ArmCommand)) {
      ArmCommand msg;
      memcpy(&msg, payload, sizeof(msg));
      msg.command[9] = '\0';
      if (enqueueCmd(msg.command))
        Serial.printf("[WS-ARM] Q+: %s\n", msg.command);
      else
        Serial.println("[WS-ARM] Queue full — drop");
    }
  }
}
// ================= BASE WEBSOCKET =================

// ================= LOCAL HTTP SERVER (fallback when base is off)
// =================
static WebServer localServer(80);

static const char ARM_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>GLITCH-ARM</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:sans-serif;background:#111;color:#eee;text-align:center;padding:10px}
h2{margin:8px 0;color:#0ff}
.st{font-size:14px;margin:4px 0;color:#888}
.on{color:#0f0}.off{color:#f44}
table{margin:8px auto;border-spacing:6px}
td{text-align:center}
button{display:block;width:64px;height:48px;font-size:15px;font-weight:bold;border:none;border-radius:8px;cursor:pointer}
.b{background:#333;color:#fff}.b:active{background:#555}
.g{background:#0a0;color:#0f0}.g:active{background:#0f0;color:#000}
.r{background:#a00;color:#f44}.r:active{background:#f44;color:#000}
.bl{background:#006;color:#6bf}.bl:active{background:#06f;color:#fff}
.y{background:#660;color:#ff0}.y:active{background:#ff0;color:#000}
.p{background:#606;color:#f0f}.p:active{background:#f0f;color:#000}
.sv{width:50px;height:36px;font-size:11px;background:#222;color:#aaa;border:1px solid #444}
.sv:active{background:#555;color:#fff}
</style></head><body>
<h2>GLITCH-ARM</h2>
<div class="st">WS: <span id="ws" class="off">DISCONNECTED</span></div>
<div class="st">Base: <span id="base" class="off">OFFLINE</span></div>
<table>
<tr><td><button class="g" onclick="cmd('GTC')">GTC</button></td>
<td><button class="bl" onclick="cmd('BTC')">BTC</button></td>
<td><button class="r" onclick="cmd('RTC')">RTC</button></td></tr>
<tr><td><button class="g" onclick="cmd('GTF')">GTF</button></td>
<td><button class="bl" onclick="cmd('BTF')">BTF</button></td>
<td><button class="r" onclick="cmd('RTF')">RTF</button></td></tr>
<tr><td><button class="y" onclick="cmd('GTP')">GTP</button></td>
<td><button class="p" onclick="cmd('BTP')">BTP</button></td>
<td><button class="y" onclick="cmd('RTP')">RTP</button></td></tr>
</table>
<table>
<tr><td><button class="g" onclick="cmd('CTP')">CTP</button></td>
<td><button class="r" onclick="cmd('H')">HOME</button></td>
<td><button class="b" onclick="cmd('S')">SCAN</button></td></tr>
</table>
<h2>Step Servos</h2>
<div style="margin:6px 0">Size: <input type="number" id="stepSize" value="5" min="1" max="90" style="width:50px;background:#222;color:#eee;border:1px solid #444;border-radius:4px;padding:3px;text-align:center">&deg;</div>
<table id="stepTable">
<tr><td class="sv">S1</td><td><button class="sv step-btn" data-idx="0" data-dir="-1">-</button></td>
<td><button class="sv step-btn" data-idx="0" data-dir="1">+</button></td></tr>
<tr><td class="sv">S2</td><td><button class="sv step-btn" data-idx="1" data-dir="-1">-</button></td>
<td><button class="sv step-btn" data-idx="1" data-dir="1">+</button></td></tr>
<tr><td class="sv">S3</td><td><button class="sv step-btn" data-idx="2" data-dir="-1">-</button></td>
<td><button class="sv step-btn" data-idx="2" data-dir="1">+</button></td></tr>
<tr><td class="sv">S4</td><td><button class="sv step-btn" data-idx="3" data-dir="-1">-</button></td>
<td><button class="sv step-btn" data-idx="3" data-dir="1">+</button></td></tr>
<tr><td class="sv">Grip</td><td><button class="sv" onclick="cmd('SV:4:-5')">-</button></td>
<td><button class="sv" onclick="cmd('SV:4:5')">+</button></td></tr>
</table>
<script>
function cmd(c){fetch('/cmd?c='+c).then(r=>r.text()).then(t=>console.log(t))}
function poll(){fetch('/status').then(r=>r.json()).then(d=>{
document.getElementById('ws').className=d.ws?'on':'off';
document.getElementById('ws').textContent=d.ws?'CONNECTED':'DISCONNECTED';
document.getElementById('base').className=d.base?'on':'off';
document.getElementById('base').textContent=d.base?'ONLINE':'OFFLINE';
}).catch(()=>{})}
setInterval(poll,1000);poll();
var stepTimer=null;
function startStep(btn){
    var idx=btn.dataset.idx, dir=parseInt(btn.dataset.dir);
    var deg=parseInt(document.getElementById('stepSize').value)||5;
    cmd('SV:'+idx+':'+(dir*deg));
    stopStep();
    stepTimer=setInterval(function(){
        var d=parseInt(document.getElementById('stepSize').value)||5;
        cmd('SV:'+idx+':'+(dir*d));
    },150);
}
function stopStep(){if(stepTimer){clearInterval(stepTimer);stepTimer=null;}}
document.querySelectorAll('.step-btn').forEach(function(btn){
    btn.addEventListener('pointerdown',function(e){e.preventDefault();startStep(btn);});
    btn.addEventListener('pointerup',stopStep);
    btn.addEventListener('pointerleave',stopStep);
    btn.addEventListener('pointercancel',stopStep);
});
</script></body></html>
)rawliteral";

static void handleRoot() { localServer.send_P(200, "text/html", ARM_PAGE); }

static void handleCmd() {
  String c = localServer.arg("c");
  if (c.length() > 0 && c.length() < 10) {
    if (enqueueCmd(c.c_str()))
      localServer.send(200, "text/plain", "OK:" + c);
    else
      localServer.send(503, "text/plain", "QUEUE FULL");
  } else {
    localServer.send(400, "text/plain", "BAD CMD");
  }
}

static void handleStatus() {
  String json = "{";
  json += "\"ws\":" + String(wsClient.isConnected() ? "true" : "false");
  json +=
      ",\"base\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false");
  json += ",\"free\":" + String(ESP.getFreeHeap());
  json += "}";
  localServer.send(200, "application/json", json);
}

static void initLocalServer() {
  localServer.on("/", handleRoot);
  localServer.on("/cmd", HTTP_GET, handleCmd);
  localServer.on("/status", HTTP_GET, handleStatus);
  localServer.begin();
  Serial.println("[ARM] Local HTTP server started on AP");
}
// ================= LOCAL HTTP SERVER =================

// ================= PCA9685 =================
Adafruit_PWMServoDriver driver = Adafruit_PWMServoDriver();
#define NUM_SERVOS 5
#define SERVOMIN 125
#define SERVOMAX 550
#define SERVO_FREQ 50

// ================= SERVO SETTINGS =================
const float GRIP_OPEN = 30.0;
const float GRIP_CLOSE = 160.0;
const float MS_PER_DEGREE = 10.0f;
const long MIN_MOVE_DURATION = 300;

// ================= WAYPOINTS =================
float posGreen[4] = {0, 100, 55, -100};
float posBlue[4] = {-100, 50, 55, -100}; // first number was -90
float posRed[4] = {100, 50, 55, -100};   // first number was 90
float posRod[4] = {0, 260, 241, 0};
float dropGreen[4] = {235, 0, 0, -90};  // right of car
float dropBlue[4] = {166, 166, 0, -90}; // diagonal right 45°
float dropRed[4] = {0, 235, 0, -90};    // front of car

// ================= GLOBAL STATE =================
float currentAngle[NUM_SERVOS] = {90, 170, 180, 100, GRIP_OPEN};
float startAngles[NUM_SERVOS];
float targetAngles[NUM_SERVOS];

// ================= GEOMETRY =================
const float L1 = 115.55, L2 = 119.76, L3 = 52.52, L4 = 105.00, L5 = 97.3;

// ================= SERVO HELPERS =================
int angleToPulse(float angle) {
  return map(constrain((int)round(angle), 0, 180), 0, 180, SERVOMIN, SERVOMAX);
}

void executeSyncMove();

void moveServo(int servoIndex, float angle) {
  if (servoIndex < 0 || servoIndex >= NUM_SERVOS)
    return;
  for (int i = 0; i < NUM_SERVOS; i++)
    targetAngles[i] = currentAngle[i];
  targetAngles[servoIndex] = angle;
  executeSyncMove();
}

// ================= INVERSE KINEMATICS =================
JointAngles calculateIK(float x, float y, float z, float phi_deg) {
  JointAngles angles;
  float phi = phi_deg * PI / 180.0;
  angles.t1 = round(atan2(y, x) * 180.0 / PI);
  float R = sqrtf(x * x + y * y);
  float Rw = R - L5 * cosf(phi);
  float Zw = z - L1 - L5 * sinf(phi);
  float R_up = sqrtf(L2 * L2 + L3 * L3);
  float d_sq = Rw * Rw + Zw * Zw;
  float cos_q3 = (d_sq - R_up * R_up - L4 * L4) / (2.0f * R_up * L4);
  if (cos_q3 < -1.0f || cos_q3 > 1.0f) {
    angles.reachable = false;
    return angles;
  }
  float q3 = acosf(cos_q3);
  float denom = R_up + L4 * cosf(q3);
  if (fabsf(denom) < 0.1f) {
    angles.reachable = false;
    return angles;
  }
  float q2 = atan2f(Zw, Rw) + atan2f(L4 * sinf(q3), denom);
  float d1 = atan2f(L3, L2);
  angles.t2 = round((q2 + d1) * (180.0 / PI));
  angles.t3 = round((q3 + d1) * (180.0 / PI));
  angles.t4 = round(90 - ((phi - q2 + q3) * 180.0 / PI));
  angles.t1 = constrain(angles.t1, 0, 180);
  angles.t2 = constrain(angles.t2, 10, 170);
  angles.t3 = constrain(angles.t3, 10, 170);
  angles.t4 = constrain(angles.t4, 50, 180);
  angles.reachable = true;
  return angles;
}

// ================= MOVE ROBOT =================
// Returns true if IK was reachable and move executed, false if unreachable.
bool moveRobot(float x, float y, float z, float pitch, int gripState) {
  JointAngles ik = calculateIK(x, y, z, pitch);
  if (!ik.reachable) {
    Serial.printf("[ARM] IK unreachable: (%.0f,%.0f,%.0f) phi=%.0f\n", x, y, z,
                  pitch);
    return false;
  }
  targetAngles[0] = ik.t1;
  targetAngles[1] = ik.t2;
  targetAngles[2] = ik.t3;
  targetAngles[3] = ik.t4;
  if (gripState != 9)
    targetAngles[4] = (gripState == 1) ? GRIP_CLOSE : GRIP_OPEN;
  executeSyncMove();
  return true;
}

void goHome() { moveRobot(0, 90, 150, -20, 0); }
void scanPose() { moveRobot(0, 150, 200, -45, 0); }

// ================= CAMERA-TO-LINK4 TRANSFORM =================
// Fixed extrinsics: camera frame -> link4 end effector frame
static const float T_CAM_TO_L4[4][4] = {{0.0f, 0.0f, 1.0f, -5.9f},
                                        {0.0f, -1.0f, 0.0f, 6.35f},
                                        {1.0f, 0.0f, 0.0f, 0.0f},
                                        {0.0f, 0.0f, 0.0f, 1.0f}};

// ================= FORWARD KINEMATICS =================
// Given joint angles t1..t4, compute link4 end effector pose (x,y,z,phi) in arm
// base frame.
static void forwardKinematics(float t1, float t2, float t3, float t4, float &x,
                              float &y, float &z, float &phi_deg) {
  float d1 = atan2f(L3, L2);
  float t1r = t1 * PI / 180.0f;
  float t2r = t2 * PI / 180.0f;
  float t3r = t3 * PI / 180.0f;
  float t4r = t4 * PI / 180.0f;

  float q2 = t2r - d1;
  float q3 = t3r - d1;

  float phi = q2 - q3 + (PI / 2.0f - t4r);

  float R_up = sqrtf(L2 * L2 + L3 * L3);

  float R_elbow = R_up * cosf(q2);
  float Z_elbow = L1 + R_up * sinf(q2);

  float R_wrist = R_elbow + L4 * cosf(q2 - q3);
  float Z_wrist = Z_elbow + L4 * sinf(q2 - q3);

  float R_ee = R_wrist + L5 * cosf(phi);
  float Z_ee = Z_wrist + L5 * sinf(phi);

  x = R_ee * cosf(t1r);
  y = R_ee * sinf(t1r);
  z = Z_ee;
  phi_deg = phi * 180.0f / PI;
}

// ================= CAMERA-FRAME -> ARM-BASE-FRAME =================
static void cameraToBase(float tx_cam, float ty_cam, float tz_cam,
                         float &x_base, float &y_base, float &z_base) {
  float l4x, l4y, l4z, l4phi;
  forwardKinematics(currentAngle[0], currentAngle[1], currentAngle[2],
                    currentAngle[3], l4x, l4y, l4z, l4phi);

  float dx = T_CAM_TO_L4[0][0] * tx_cam + T_CAM_TO_L4[0][1] * ty_cam +
             T_CAM_TO_L4[0][2] * tz_cam + T_CAM_TO_L4[0][3];
  float dy = T_CAM_TO_L4[1][0] * tx_cam + T_CAM_TO_L4[1][1] * ty_cam +
             T_CAM_TO_L4[1][2] * tz_cam + T_CAM_TO_L4[1][3];
  float dz = T_CAM_TO_L4[2][0] * tx_cam + T_CAM_TO_L4[2][1] * ty_cam +
             T_CAM_TO_L4[2][2] * tz_cam + T_CAM_TO_L4[2][3];

  float t1r = currentAngle[0] * PI / 180.0f;
  x_base = l4x + dx * cosf(t1r) - dy * sinf(t1r);
  y_base = l4y + dx * sinf(t1r) + dy * cosf(t1r);
  z_base = l4z + dz;
}

// Forward declarations for color-based place functions
void GreenToCar();
void BlueToCar();
void RedToCar();
void GreenToPlatform();
void BlueToPlatform();
void RedToPlatform();

// ================= CAMERA-GUIDED PICKUP =================
// Called from loop() when camera pose data arrives via ESP-NOW.
// Transforms QR from camera frame -> arm base frame -> IK -> grip -> color
// place.
static void cameraGuidedPickup() {
  CameraPoseData *p = &incomingCameraPose;

  if (!p->pose_valid) {
    Serial.println("[CAM] Ignoring invalid pose");
    return;
  }

  // Move home first so currentAngle[] is known for cameraToBase FK
  goHome();

  float alpha_deg = currentAngle[1] - currentAngle[2];
  Serial.printf("[CAM] Guided pickup: color=%d conf=%.2f "
                "qr=(%.0f,%.0f,%.0f) yaw=%.1f alpha=%.1f\n",
                p->color, p->confidence, p->tx_mm, p->ty_mm, p->tz_mm,
                p->yaw_deg, alpha_deg);
  Serial.printf("[CAM] Joint angles: [%.0f, %.0f, %.0f, %.0f]\n",
                currentAngle[0], currentAngle[1], currentAngle[2],
                currentAngle[3]);

  // 1. Convert QR position from camera frame -> arm base frame
  float x_qr, y_qr, z_qr;
  cameraToBase(p->tx_mm, p->ty_mm, p->tz_mm, x_qr, y_qr, z_qr);
  Serial.printf("[CAM] QR in base frame: (%.0f, %.0f, %.0f)\n", x_qr, y_qr,
                z_qr);

  // 2. Approach: move 40mm above QR with gripper open
  if (!moveRobot(x_qr, y_qr, z_qr + 40, -90, 0)) {
    Serial.println("[CAM] ABORT: approach above QR unreachable");
    goHome();
    return;
  }

  // 3. Descend to QR
  if (!moveRobot(x_qr, y_qr, z_qr, -90, 0)) {
    Serial.println("[CAM] ABORT: descend to QR unreachable");
    goHome();
    return;
  }

  // 4. Close gripper
  moveRobot(x_qr, y_qr, z_qr, -90, 1);
  delay(200);

  // 5. Lift QR
  if (!moveRobot(x_qr, y_qr, z_qr + 40, -90, 1)) {
    Serial.println("[CAM] ABORT: lift unreachable");
    goHome();
    return;
  }

  // 6. Go home with object
  goHome();

  // 7. Place based on color
  switch (p->color) {
  case 1:
    Serial.println("[CAM] -> RedToCar");
    RedToCar();
    break;
  case 2:
    Serial.println("[CAM] -> GreenToCar");
    GreenToCar();
    break;
  case 3:
    Serial.println("[CAM] -> BlueToCar");
    BlueToCar();
    break;
  default:
    Serial.printf("[CAM] Unknown color %d, going home\n", p->color);
    goHome();
    break;
  }

  Serial.println("[CAM] Camera-guided pickup complete");
}

// ================= EXECUTE SYNC MOVE =================
// Blocking but yields every 10 ms via delay().  During the yield, the ESP-NOW
// callback can fire and fill the command queue.  loop() will drain it
// when this function (and its caller) returns.
void executeSyncMove() {
  unsigned long startTime = millis();
  float maxDelta = 0.0f;
  for (int i = 0; i < NUM_SERVOS; i++) {
    startAngles[i] = currentAngle[i];
    float delta = fabsf(targetAngles[i] - currentAngle[i]);
    if (delta > maxDelta)
      maxDelta = delta;
  }
  long duration = (long)(maxDelta * MS_PER_DEGREE);
  if (duration < MIN_MOVE_DURATION)
    duration = MIN_MOVE_DURATION;

  while (true) {
    unsigned long elapsed = millis() - startTime;
    if (elapsed >= (unsigned long)duration)
      break;

    float progress = (float)elapsed / (float)duration;
    float smoothStep = (1.0f - cosf(progress * PI)) / 2.0f;

    for (int i = 0; i < NUM_SERVOS; i++) {
      float travel = targetAngles[i] - startAngles[i];
      currentAngle[i] = startAngles[i] + (travel * smoothStep);
      driver.setPWM(i, 0, angleToPulse(currentAngle[i]));
    }
    delay(10); // yields to FreeRTOS — WiFi task runs, queue fills
  }

  for (int i = 0; i < NUM_SERVOS; i++) {
    currentAngle[i] = targetAngles[i];
    driver.setPWM(i, 0, angleToPulse(currentAngle[i]));
  }
}

// ================= STEP SERVO =================
// For joints 0-3: dir = signed step in degrees (e.g., +5 or -10)
// For joint 4 (claw): toggles between GRIP_OPEN and GRIP_CLOSE
void stepServo(int idx, int dir) {
  if (idx < 0 || idx >= NUM_SERVOS) {
    Serial.printf("[ARM] stepServo: invalid idx %d\n", idx);
    return;
  }
  if (idx == 4) {
    // Claw toggle — just hold or release
    float target = (currentAngle[4] > (GRIP_OPEN + GRIP_CLOSE) / 2.0f)
                       ? GRIP_OPEN
                       : GRIP_CLOSE;
    Serial.printf("[ARM] Claw toggle: %.0f -> %.0f\n", currentAngle[4], target);
    moveServo(4, target);
    return;
  }
  float newAng = currentAngle[idx] + (float)dir;
  newAng = constrain(newAng, 0.0f, 180.0f);
  Serial.printf("[ARM] stepServo S%d: %.0f -> %.0f (dir=%d)\n", idx + 1,
                currentAngle[idx], newAng, dir);
  moveServo(idx, newAng);
}

// ================= AUTOMATED SEQUENCES =================
// Each blocks during execution but yields every 10 ms (via executeSyncMove).
// The command queue absorbs ESP-NOW packets during a move.
// delay(200) after gripper close gives the physical gripper time to seat.

void GreenToFloor() {
  moveRobot(posGreen[0], posGreen[1], posGreen[2] + 40, posGreen[3], 0);
  moveRobot(posGreen[0], posGreen[1], posGreen[2], posGreen[3], 0);
  moveRobot(posGreen[0], posGreen[1], posGreen[2], posGreen[3], 1);
  delay(200);
  moveRobot(posGreen[0], posGreen[1], posGreen[2] + 40, posGreen[3], 1);
  moveRobot(dropGreen[0], dropGreen[1], dropGreen[2], dropGreen[3], 1);
  moveRobot(dropGreen[0], dropGreen[1], dropGreen[2], dropGreen[3], 0);
  delay(200);
  goHome();
}

void BlueToFloor() {
  moveRobot(posBlue[0], posBlue[1], posBlue[2] + 40, posBlue[3], 0);
  moveRobot(posBlue[0], posBlue[1], posBlue[2], posBlue[3], 0);
  moveRobot(posBlue[0], posBlue[1], posBlue[2], posBlue[3], 1);
  delay(200);
  moveRobot(posBlue[0], posBlue[1], posBlue[2] + 40, posBlue[3], 1);
  moveRobot(dropBlue[0], dropBlue[1], dropBlue[2], dropBlue[3], 1);
  moveRobot(dropBlue[0], dropBlue[1], dropBlue[2], dropBlue[3], 0);
  delay(200);
  goHome();
}

void RedToFloor() {
  moveRobot(posRed[0], posRed[1], posRed[2] + 40, posRed[3], 0);
  moveRobot(posRed[0], posRed[1], posRed[2], posRed[3], 0);
  moveRobot(posRed[0], posRed[1], posRed[2], posRed[3], 1);
  delay(200);
  moveRobot(posRed[0], posRed[1], posRed[2] + 40, posRed[3], 1);
  moveRobot(dropRed[0], dropRed[1], dropRed[2], dropRed[3], 1);
  moveRobot(dropRed[0], dropRed[1], dropRed[2], dropRed[3], 0);
  delay(200);
  goHome();
}

void GreenToCar() {
  moveRobot(posRod[0], posRod[1], posRod[2], posRod[3], 1);
  moveRobot(posRod[0], posRod[1], posRod[2] + 40, posRod[3], 1);
  moveRobot(posRod[0], posRod[1] - 60, posRod[2] + 40, posRod[3], 1);
  moveRobot(posGreen[0], posGreen[1], posGreen[2] + 40, posGreen[3], 1);
  moveRobot(posGreen[0], posGreen[1] + 20, posGreen[2], posGreen[3], 1);
  moveRobot(posGreen[0], posGreen[1], posGreen[2], posGreen[3], 0);
  moveRobot(posGreen[0], posGreen[1], posGreen[2] + 40, posGreen[3], 0);
  delay(200);
  goHome();
}
// msh mzboot
void BlueToCar() {
  moveRobot(posRod[0], posRod[1], posRod[2], posRod[3], 1);
  moveRobot(posRod[0], posRod[1], posRod[2] + 40, posRod[3], 1);
  moveRobot(posRod[0], posRod[1] - 60, posRod[2] + 40, posRod[3], 1);
  moveRobot(posBlue[0], posBlue[1], posBlue[2] + 40, posBlue[3], 1);
  moveRobot(posBlue[0] - 45, posBlue[1] + 15, posBlue[2], posBlue[3], 1);
  moveRobot(posBlue[0], posBlue[1], posBlue[2], posBlue[3], 0);
  moveRobot(posBlue[0], posBlue[1], posBlue[2] + 40, posBlue[3], 0);
  delay(200);
  goHome();
}

void RedToCar() {
  moveRobot(posRod[0], posRod[1], posRod[2], posRod[3], 1);
  moveRobot(posRod[0], posRod[1], posRod[2] + 40, posRod[3], 1);
  moveRobot(posRod[0], posRod[1] - 60, posRod[2] + 40, posRod[3], 1);
  moveRobot(posRed[0], posRed[1], posRed[2] + 40, posRed[3], 1);
  moveRobot(posRed[0] + 30, posRed[1] + 30, posRed[2], posRed[3], 1);
  moveRobot(posRed[0], posRed[1], posRed[2], posRed[3], 0);
  moveRobot(posRed[0], posRed[1], posRed[2] + 40, posRed[3], 0);
  delay(200);
  goHome();
}

// ================= COLOR TO PLATFORM =================
// Pick from color position, place on platform (posRod).

void GreenToPlatform() {
  moveRobot(posGreen[0], posGreen[1], posGreen[2] + 40, posGreen[3], 0);
  moveRobot(posGreen[0], posGreen[1], posGreen[2], posGreen[3], 0);
  moveRobot(posGreen[0], posGreen[1], posGreen[2], posGreen[3], 1);
  delay(200);
  moveRobot(posGreen[0], posGreen[1], posGreen[2] + 40, posGreen[3], 1);
  moveRobot(0, 90, 150, -20, 1);
  moveRobot(posRod[0], posRod[1], posRod[2] + 40, posRod[3], 1);
  moveRobot(posRod[0], posRod[1], posRod[2], posRod[3], 1);
  moveRobot(posRod[0], posRod[1], posRod[2], posRod[3], 0);
  delay(200);
  moveRobot(posRod[0], posRod[1], posRod[2] + 40, posRod[3], 0);
  goHome();
}

void BlueToPlatform() {
  moveRobot(posBlue[0], posBlue[1], posBlue[2] + 40, posBlue[3], 0);
  moveRobot(posBlue[0], posBlue[1], posBlue[2], posBlue[3], 0);
  moveRobot(posBlue[0], posBlue[1], posBlue[2], posBlue[3], 1);
  delay(200);
  moveRobot(posBlue[0], posBlue[1], posBlue[2] + 40, posBlue[3], 1);
  moveRobot(0, 90, 150, -20, 1);
  moveRobot(posRod[0], posRod[1], posRod[2] + 40, posRod[3], 1);
  moveRobot(posRod[0], posRod[1], posRod[2], posRod[3], 1);
  moveRobot(posRod[0], posRod[1], posRod[2], posRod[3], 0);
  delay(200);
  moveRobot(posRod[0], posRod[1], posRod[2] + 40, posRod[3], 0);
  goHome();
}

void RedToPlatform() {
  moveRobot(posRed[0], posRed[1], posRed[2] + 40, posRed[3], 0);
  moveRobot(posRed[0], posRed[1], posRed[2], posRed[3], 0);
  moveRobot(posRed[0], posRed[1], posRed[2], posRed[3], 1);
  delay(200);
  moveRobot(posRed[0], posRed[1], posRed[2] + 40, posRed[3], 1);
  moveRobot(0, 90, 150, -20, 1);
  moveRobot(posRod[0], posRod[1], posRod[2] + 40, posRod[3], 1);
  moveRobot(posRod[0], posRod[1], posRod[2], posRod[3], 1);
  moveRobot(posRod[0], posRod[1], posRod[2], posRod[3], 0);
  delay(200);
  moveRobot(posRod[0], posRod[1], posRod[2] + 40, posRod[3], 0);
  goHome();
}

// ================= CAR TO PLATFORM (manual pickup) =================
// Approach sequence: moves to posRod with gripper open.
// User then manually adjusts with step buttons and grips.
void CarToPlatform() {
  Serial.println("[ACTION] Car To Platform — approach");
  moveRobot(posRod[0], posRod[1], posRod[2] + 40, posRod[3], 0); // was +40
  moveRobot(posRod[0], posRod[1], posRod[2], posRod[3], 0);
  Serial.println("[ACTION] At car — use step buttons to adjust, then grip");
}

// ================= DISPATCH (called from loop, NEVER from callback)
// =================
static void sendArmStatus(bool busy) {
  ArmStatus st = {};
  st.type = 0;
  st.busy = busy ? 1 : 0;
  if (wsClient.isConnected()) {
    wsClient.sendBIN((uint8_t *)&st, sizeof(st));
  } else {
    Serial.println("[ARM] WS not connected — status dropped");
  }
}

static void dispatchCmd(const char *cmd) {
  if (strcmp(cmd, "GTC") == 0)
    GreenToCar();
  else if (strcmp(cmd, "BTC") == 0)
    BlueToCar();
  else if (strcmp(cmd, "RTC") == 0)
    RedToCar();
  else if (strcmp(cmd, "GTF") == 0)
    GreenToFloor();
  else if (strcmp(cmd, "BTF") == 0)
    BlueToFloor();
  else if (strcmp(cmd, "RTF") == 0)
    RedToFloor();
  else if (strcmp(cmd, "CTP") == 0)
    CarToPlatform();
  else if (strcmp(cmd, "GTP") == 0)
    GreenToPlatform();
  else if (strcmp(cmd, "RTP") == 0)
    RedToPlatform();
  else if (strcmp(cmd, "BTP") == 0)
    BlueToPlatform();
  else if (strcmp(cmd, "H") == 0)
    goHome();
  else if (strcmp(cmd, "S") == 0)
    scanPose();
  else if (strncmp(cmd, "SV:", 3) == 0) {
    int idx = -1, dir = 0;
    if (sscanf(cmd, "SV:%d:%d", &idx, &dir) == 2)
      stepServo(idx, dir);
  } else
    Serial.printf("[ARM] Unknown: %s\n", cmd);
}

// ================= WIFI RECONNECT (throttled) =================
// Only attempts once every 5 s.  Does NOT block.
static unsigned long lastWifiAttempt = 0;

static void checkWifi() {
  bool isConnected = (WiFi.status() == WL_CONNECTED);

  if (isConnected)
    return;
  if (millis() - lastWifiAttempt < 5000)
    return;
  lastWifiAttempt = millis();
  Serial.println("[ARM] STA down, reconnecting to base...");
  WiFi.begin("GLITCH", "Gl1tch2024!Secure");
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  // I2C + PCA9685 init (matches Blynk reference — clean, no retry/probe)
  Wire.begin(21, 22);

  driver.begin();
  driver.setOscillatorFrequency(27000000);
  driver.setPWMFreq(SERVO_FREQ);

  for (int i = 0; i < NUM_SERVOS; i++) {
    driver.setPWM(i, 0, angleToPulse(currentAngle[i]));
  }

  delay(1000);
  Serial.println("[ARM] PCA9685 init done");

  // WiFi AP+STA mode — own AP always on, STA tries to connect to base
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(IPAddress(192, 168, 5, 1), IPAddress(192, 168, 5, 1),
                    IPAddress(255, 255, 255, 0));
  WiFi.softAP("GLITCH-ARM", "Gl1tch2024!Secure", 11);
  Serial.printf("[ARM] AP: %s ch=11\n", WiFi.softAPIP().toString().c_str());

  // STA: try to connect to base AP (non-blocking after timeout)
  WiFi.begin("GLITCH", "Gl1tch2024!Secure");
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 5000) {
    delay(100);
  }
  esp_wifi_set_ps(WIFI_PS_NONE);

  Serial.printf("[ARM] MAC: %s  STA: %s  AP: %s\n", WiFi.macAddress().c_str(),
                (WiFi.status() == WL_CONNECTED) ? "OK" : "DOWN",
                WiFi.softAPIP().toString().c_str());

  // WebSocket client — connects to base AP if available
  wsClient.begin(BASE_WS_HOST, BASE_WS_PORT, "/");
  wsClient.onEvent(onWsEvent);

  // Local HTTP server — always available via AP
  initLocalServer();

  Serial.println("[ARM] Ready — AP+STA, WebSocket + local HTTP");
}

// ================= LOOP =================
void loop() {
  static bool homed = false;
  if (!homed) {
    homed = true;
    goHome();
  }

  if (WiFi.status() == WL_CONNECTED) {
    wsClient.loop(); // Only poll WebSocket when STA is up
  }
  localServer.handleClient(); // Always process local HTTP requests
  checkWifi();

  static unsigned long lastStatus = 0;
  if (millis() - lastStatus > 5000) {
    lastStatus = millis();
    Serial.printf("[ARM] STA=%d WS=%d AP=%s free=%d\n",
                  WiFi.status() == WL_CONNECTED ? 1 : 0,
                  wsClient.isConnected() ? 1 : 0,
                  WiFi.softAPIP().toString().c_str(), ESP.getFreeHeap());
    // Heartbeat: send idle status so base knows arm is alive
    if (WiFi.status() == WL_CONNECTED) {
      sendArmStatus(false);
    }
  }

  // Drain commands.  Send busy ONCE before batch, idle ONCE after.
  bool didWork = false;
  for (int i = 0; i < 4; i++) {
    char cmd[10];
    if (!dequeueCmd(cmd))
      break;
    if (!didWork) {
      sendArmStatus(true);
      didWork = true;
    }
    Serial.printf("[ARM] Run: %s\n", cmd);
    dispatchCmd(cmd);
  }
  if (didWork) {
    sendArmStatus(false);
  }

  // If queue is empty, yield explicitly so the WiFi task gets CPU
  delay(10);
}
