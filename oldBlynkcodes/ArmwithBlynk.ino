// =========================== ARM ESP32 CODE ===========================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <esp_now.h>
#include <WiFi.h>

// ================= WIFI =================
char ssid[] = "hassan's-laptop-hotspot";
char pass[] = "12345678";

// ================= PCA9685 =================
Adafruit_PWMServoDriver driver = Adafruit_PWMServoDriver();

#define NUM_SERVOS 5

// ================= ESP NOW =================

// Base ESP32 MAC Address
uint8_t baseAddress[] = {0x80, 0xF3, 0xDA, 0x42, 0x3E, 0x5C};

typedef struct struct_message {
  char command[10];
} struct_message;

struct_message incomingMessage;

// Camera pose data forwarded by base via ESP-NOW (size differs from struct_message)
struct __attribute__((packed)) CameraPoseData {
    uint8_t type;        // 1 = camera pose packet
    uint8_t pose_valid;
    uint8_t color;
    uint8_t estimated;
    float tx_mm;
    float ty_mm;
    float tz_mm;
    float yaw_deg;
    float confidence;
};

static CameraPoseData incomingCameraPose;

// --- GRIPPER SETTINGS ---
#define GRIP_CH 4

const float GRIP_OPEN  = 30.0;
const float GRIP_CLOSE = 160.0;

// --- PRE-DEFINED WAYPOINTS ---
float posGreen[4]   = {0,  100, 55, -100};
float posBlue[4]    = {-90, 50, 55, -100};
float posRed[4]     = {90, 50, 55, -100};
float posRod[4]     = {0,  260, 200, 0};

float dropGreen[4]  = {0, 235, 0, -90};
float dropBlue[4]   = {0, 235, 0, -90};
float dropRed[4]    = {-235, 0, 0, -90};

// --- GLOBAL TRACKING VARIABLES ---
float currentAngle[NUM_SERVOS] = {90, 170, 180, 100, GRIP_OPEN};

float startAngles[NUM_SERVOS];
float targetAngles[NUM_SERVOS];

const float MS_PER_DEGREE = 10.0f;
const long MIN_MOVE_DURATION = 300;

#define SERVOMIN  125
#define SERVOMAX  550
#define SERVO_FREQ 50

// --- ROBOT GEOMETRY ---
const float L1 = 115.55;
const float L2 = 119.76;
const float L3 = 52.52;
const float L4 = 105.00;
const float L5 = 97.3;

// ================================================================
// CAMERA-TO-LINK4 TRANSFORM
// ================================================================
// Fixed extrinsics: camera frame ΓåÆ link4 end effector frame (given by calibration)
// Transforms a point P_cam in camera frame to P_l4 in link4 frame:
//   [P_l4] = T_CAM_TO_L4 * [P_cam]
// Axes mapping: x_camΓåÆz_l4, y_camΓåÆ-y_l4, z_camΓåÆx_l4
// Translation: camera origin is at (-5.9, 6.35, 0) in link4 frame
static const float T_CAM_TO_L4[4][4] = {
    { 0.0f,  0.0f,  1.0f,  -5.9f    },
    { 0.0f, -1.0f,  0.0f,   6.35f   },
    { 1.0f,  0.0f,  0.0f,   0.0f    },
    { 0.0f,  0.0f,  0.0f,   1.0f    }
};

// ================================================================
// STRUCT
// ================================================================

struct JointAngles {
  int t1, t2, t3, t4;
  bool reachable;
};

// ================================================================
// HELPER FUNCTIONS
// ================================================================

int angleToPulse(float angle) {

  return map(constrain(angle, 0, 180),
             0, 180,
             SERVOMIN, SERVOMAX);
}

// ================================================================
// EXECUTE SYNCHRONIZED MOVE
// ================================================================

void executeSyncMove() {

  unsigned long startTime = millis();
 
  float maxDelta = 0;
  for(int i = 0; i < 5; i++) {
    startAngles[i] = currentAngle[i];
    float delta = fabs(targetAngles[i] - currentAngle[i]);
    if (delta > maxDelta) maxDelta = delta;
  }
  
  long duration = (long)(maxDelta * MS_PER_DEGREE);
  if (duration < MIN_MOVE_DURATION) duration = MIN_MOVE_DURATION;

  while (true) {

    unsigned long elapsed = millis() - startTime;

    if (elapsed >= duration)
      break;

    float progress = (float)elapsed / (float)duration;

    float smoothStep = (1.0 - cos(progress * PI)) / 2.0;

    for (int i = 0; i < 5; i++) {

      float travel = targetAngles[i] - startAngles[i];

      currentAngle[i] = startAngles[i] + (travel * smoothStep);

      driver.setPWM(i, 0, angleToPulse(currentAngle[i]));
    }

    delay(10);
  }

  for (int i = 0; i < 5; i++) {

    currentAngle[i] = targetAngles[i];

    driver.setPWM(i, 0, angleToPulse(currentAngle[i]));
  }
}

// ================================================================
// INVERSE KINEMATICS
// ================================================================

JointAngles calculateIK(float x, float y, float z, float phi_deg) {

  JointAngles angles;

  float phi = phi_deg * PI / 180.0;

  angles.t1 = round(atan2(y, x) * 180.0 / PI);

  float R = sqrt(pow(x, 2) + pow(y, 2));

  float Rw = R - L5 * cos(phi);

  float Zw = z - L1 - L5 * sin(phi);

  float R_up = sqrt(pow(L2, 2) + pow(L3, 2));

  float d_sq = pow(Rw, 2) + pow(Zw, 2);

  float cos_q3 =
    (d_sq - pow(R_up, 2) - pow(L4, 2)) /
    (2 * R_up * L4);

  if (cos_q3 < -1.0 || cos_q3 > 1.0) {
    angles.reachable = false;
    return angles;
  }

  float q3 = acos(cos_q3);

  float denom = R_up + L4 * cos(q3);
  if (fabs(denom) < 0.1f) {
    angles.reachable = false;
    return angles;
  }

  float q2 =
    atan2(Zw, Rw) +
    atan2(L4 * sin(q3), denom);

  float d1 = atan2(L3, L2);

  angles.t2 = round((q2 + d1) * (180.0 / PI));

  angles.t3 = round((q3 + d1) * (180.0 / PI));

  angles.t4 = round(90 - ((phi - q2 + q3) * 180.0 / PI));

  if (angles.t1 < 0 || angles.t1 > 180 ||
      angles.t2 < 10 || angles.t2 > 170 ||
      angles.t3 < 10 || angles.t3 > 170 ||
      angles.t4 < 50 || angles.t4 > 180) {
    angles.reachable = false;
    return angles;
  }

  angles.t1 = constrain(angles.t1, 0, 180);
  angles.t2 = constrain(angles.t2, 10, 170);
  angles.t3 = constrain(angles.t3, 10, 170);
  angles.t4 = constrain(angles.t4, 50, 180);

  angles.reachable = true;

  return angles;
}

// ================================================================
// MOVE SINGLE SERVO
// ================================================================

void moveServo(int servoIndex, float angle) {
  if (servoIndex < 0 || servoIndex >= NUM_SERVOS) {
    Serial.println("[!] ERROR: Invalid Servo Index");
    return;
  }

  Serial.printf("\n--------------------------------------------------\n");
  Serial.printf("INPUT -> Move Servo %d to %.1f\n", servoIndex, angle);

  for (int i = 0; i < NUM_SERVOS; i++) {
    targetAngles[i] = currentAngle[i]; 
  }

  targetAngles[servoIndex] = angle;

  executeSyncMove();

  Serial.printf("REACHED -> T1:%.1f T2:%.1f T3:%.1f T4:%.1f G:%.1f\n",
                currentAngle[0],
                currentAngle[1],
                currentAngle[2],
                currentAngle[3],
                currentAngle[4]);
  Serial.println("--------------------------------------------------");
}

// ================================================================
// FORWARD KINEMATICS
// ================================================================
// Given joint angles t1..t4, compute link4 end effector pose (x,y,z,phi) in arm base frame.
// phi = tool pitch angle from horizontal (degrees).
static void forwardKinematics(float t1, float t2, float t3, float t4,
                               float &x, float &y, float &z, float &phi_deg) {
  float d1  = atan2(L3, L2);
  float t1r = t1 * PI / 180.0f;
  float t2r = t2 * PI / 180.0f;
  float t3r = t3 * PI / 180.0f;
  float t4r = t4 * PI / 180.0f;

  float q2 = t2r - d1;
  float q3 = t3r - d1;

  float phi = q2 - q3 + (PI/2.0f - t4r);

  float R_up = sqrt(L2*L2 + L3*L3);

  float R_elbow = R_up * cos(q2);
  float Z_elbow = L1 + R_up * sin(q2);

  float R_wrist = R_elbow + L4 * cos(q2 - q3);
  float Z_wrist = Z_elbow + L4 * sin(q2 - q3);

  float R_ee = R_wrist + L5 * cos(phi);
  float Z_ee = Z_wrist + L5 * sin(phi);

  x = R_ee * cos(t1r);
  y = R_ee * sin(t1r);
  z = Z_ee;
  phi_deg = phi * 180.0f / PI;
}

// ================================================================
// CAMERA-FRAME ΓåÆ ARM-BASE-FRAME TRANSFORM
// ================================================================
static void cameraToBase(float tx_cam, float ty_cam, float tz_cam,
                          float &x_base, float &y_base, float &z_base) {
  float l4x, l4y, l4z, l4phi;
  forwardKinematics(currentAngle[0], currentAngle[1],
                    currentAngle[2], currentAngle[3],
                    l4x, l4y, l4z, l4phi);

  float dx = T_CAM_TO_L4[0][0]*tx_cam + T_CAM_TO_L4[0][1]*ty_cam +
             T_CAM_TO_L4[0][2]*tz_cam + T_CAM_TO_L4[0][3];
  float dy = T_CAM_TO_L4[1][0]*tx_cam + T_CAM_TO_L4[1][1]*ty_cam +
             T_CAM_TO_L4[1][2]*tz_cam + T_CAM_TO_L4[1][3];
  float dz = T_CAM_TO_L4[2][0]*tx_cam + T_CAM_TO_L4[2][1]*ty_cam +
             T_CAM_TO_L4[2][2]*tz_cam + T_CAM_TO_L4[2][3];

  float t1r = currentAngle[0] * PI / 180.0f;
  x_base = l4x + dx*cos(t1r) - dy*sin(t1r);
  y_base = l4y + dx*sin(t1r) + dy*cos(t1r);
  z_base = l4z + dz;
}

// ================================================================
// MOVE ROBOT
// ================================================================

void moveRobot(float x,
               float y,
               float z,
               float pitch,
               int gripState) {

  JointAngles ik = calculateIK(x, y, z, pitch);

  if (ik.reachable) {

    Serial.println("\n--------------------------------------------------");

    Serial.printf("INPUT -> X:%.1f Y:%.1f Z:%.1f Phi:%.1f Grip:%d\n",
                  x, y, z, pitch, gripState);

    targetAngles[0] = ik.t1;
    targetAngles[1] = ik.t2;
    targetAngles[2] = ik.t3;
    targetAngles[3] = ik.t4;

    if (gripState != 9)
      targetAngles[4] =
        (gripState == 1) ? GRIP_CLOSE : GRIP_OPEN;

    Serial.printf("TARGETS -> T1:%d T2:%d T3:%d T4:%d G:%.0f\n",
                  ik.t1, ik.t2, ik.t3, ik.t4, targetAngles[4]);

    executeSyncMove();

    Serial.printf("REACHED -> T1:%.1f T2:%.1f T3:%.1f T4:%.1f G:%.1f\n",
                  currentAngle[0],
                  currentAngle[1],
                  currentAngle[2],
                  currentAngle[3],
                  currentAngle[4]);

    Serial.println("--------------------------------------------------");
  }
  else {

    Serial.printf("\n[!] ERROR: Unreachable "
                  "(X:%.1f Y:%.1f Z:%.1f)\n",
                  x, y, z);
  }
}

// ================================================================
// HOME POSITION
// ================================================================

void goHome() {

  Serial.println("\n[SYSTEM] Returning Home");

  moveRobot(0, 90, 150, -20, 0);
}

void scanPose() {

  Serial.println("\n[SYSTEM] Moving to Scan Pose");

  moveRobot(0, 150, 200, -45, 0);
}

// ================================================================
// CAMERA-GUIDED PICKUP
// ================================================================
// Called when camera pose data arrives via ESP-NOW.
// Transforms QR from camera frame ΓåÆ arm base frame ΓåÆ IK ΓåÆ execute.
static void cameraGuidedPickup() {
  CameraPoseData *p = &incomingCameraPose;

  if (!p->pose_valid) {
    Serial.println("[CAM] Ignoring invalid pose");
    return;
  }

  Serial.printf("[CAM] Guided pickup: color=%d conf=%.2f "
                "qr=(%.0f,%.0f,%.0f) yaw=%.1f\n",
                p->color, p->confidence,
                p->tx_mm, p->ty_mm, p->tz_mm, p->yaw_deg);

  // 1. Convert QR position from camera frame ΓåÆ arm base frame
  float x_qr, y_qr, z_qr;
  cameraToBase(p->tx_mm, p->ty_mm, p->tz_mm, x_qr, y_qr, z_qr);

  Serial.printf("[CAM] QR in base frame: (%.0f, %.0f, %.0f)\n",
                x_qr, y_qr, z_qr);

  // 2. Approach: move 40mm above QR with gripper open
  moveRobot(x_qr, y_qr, z_qr + 40, -90, 0);

  // 3. Descend to QR
  moveRobot(x_qr, y_qr, z_qr, -90, 0);

  // 4. Close gripper
  moveRobot(x_qr, y_qr, z_qr, -90, 1);

  delay(200);

  // 5. Lift QR
  moveRobot(x_qr, y_qr, z_qr + 40, -90, 1);

  // 6. Go to home with object
  goHome();

  // 7. TODO: add drop-off based on p->color to a dedicated location
  //    (see GreenToFloor, BlueToFloor, RedToFloor for color-based drops)

  Serial.println("[CAM] Camera-guided pickup complete");
}

// ================================================================
// AUTOMATED FUNCTIONS
// ================================================================

void GreenToFloor() {

  Serial.println("[ACTION] Green To Floor");

  moveRobot(posGreen[0], posGreen[1], posGreen[2]+40, posGreen[3], 0);
  moveRobot(posGreen[0], posGreen[1], posGreen[2], posGreen[3], 0);
  moveRobot(posGreen[0], posGreen[1], posGreen[2], posGreen[3], 1);

  delay(200);

  moveRobot(posGreen[0], posGreen[1], posGreen[2]+40, posGreen[3], 1);
  moveRobot(dropGreen[0], dropGreen[1], dropGreen[2], dropGreen[3], 1);
  moveRobot(dropGreen[0], dropGreen[1], dropGreen[2], dropGreen[3], 0);

  delay(200);

  goHome();
}

void BlueToFloor() {

  Serial.println("[ACTION] Blue To Floor");

  moveRobot(posBlue[0], posBlue[1], posBlue[2]+40, posBlue[3], 0);
  moveRobot(posBlue[0], posBlue[1], posBlue[2], posBlue[3], 0);
  moveRobot(posBlue[0], posBlue[1], posBlue[2], posBlue[3], 1);

  delay(200);

  moveRobot(posBlue[0], posBlue[1], posBlue[2]+40, posBlue[3], 1);
  moveRobot(dropBlue[0], dropBlue[1], dropBlue[2], dropBlue[3], 1);
  moveRobot(dropBlue[0], dropBlue[1], dropBlue[2], dropBlue[3], 0);

  delay(200);

  goHome();
}

void RedToFloor() {

  Serial.println("[ACTION] Red To Floor");

  moveRobot(posRed[0], posRed[1], posRed[2]+40, posRed[3], 0);
  moveRobot(posRed[0], posRed[1], posRed[2], posRed[3], 0);
  moveRobot(posRed[0], posRed[1], posRed[2], posRed[3], 1);

  delay(200);

  moveRobot(posRed[0], posRed[1], posRed[2]+40, posRed[3], 1);
  moveRobot(dropRed[0], dropRed[1], dropRed[2], dropRed[3], 1);
  moveRobot(dropRed[0], dropRed[1], dropRed[2], dropRed[3], 0);

  delay(200);

  goHome();
}

void GreenToCar() {

  Serial.println("[ACTION] Green To Car");

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

void BlueToCar() {

  Serial.println("[ACTION] Blue To Car");

  moveRobot(posRod[0], posRod[1], posRod[2], posRod[3], 1);

  moveRobot(posRod[0], posRod[1], posRod[2] + 40, posRod[3], 1);

  moveRobot(posRod[0], posRod[1] - 60, posRod[2] + 40, posRod[3], 1);

  moveRobot(posBlue[0], posBlue[1], posBlue[2] + 40, posBlue[3], 1);

  moveRobot(posBlue[0] - 20, posBlue[1] + 20, posBlue[2], posBlue[3], 1);

  moveRobot(posBlue[0], posBlue[1], posBlue[2], posBlue[3], 0);

  moveRobot(posBlue[0], posBlue[1], posBlue[2] + 40, posBlue[3], 0);

  delay(200);

  goHome();
}

void RedToCar() {

  Serial.println("[ACTION] Red To Car");

  moveRobot(posRod[0], posRod[1], posRod[2], posRod[3], 1);

  moveRobot(posRod[0], posRod[1], posRod[2] + 40, posRod[3], 1);

  moveRobot(posRod[0], posRod[1] - 60, posRod[2] + 40, posRod[3], 1);

  moveRobot(posRed[0], posRed[1], posRed[2] + 40, posRed[3], 1);

  moveRobot(posRed[0] + 20, posRed[1] + 20, posRed[2], posRed[3], 1);

  moveRobot(posRed[0], posRed[1], posRed[2], posRed[3], 0);

  moveRobot(posRed[0], posRed[1], posRed[2] + 40, posRed[3], 0);

  delay(200);

  goHome();
}

void FromCarToRod() {

  Serial.println("[ACTION] From Car To Rod");

  moveRobot(posRod[0], posRod[1], posRod[2] + 40, posRod[3], 0);
  moveRobot(posRod[0], posRod[1], posRod[2], posRod[3], 0);

}


// ================================================================
// ESP NOW RECEIVE CALLBACK
// ================================================================

void OnDataRecv(const esp_now_recv_info_t *recvInfo,
                const uint8_t *incomingData,
                int len) {

  // Camera pose data packet (larger than command string)
  if (len == sizeof(CameraPoseData)) {
    memcpy(&incomingCameraPose, incomingData, sizeof(CameraPoseData));

    Serial.println("\n==============================");
    Serial.print("ESP-NOW CAMERA POSE RECEIVED: valid=");
    Serial.print(incomingCameraPose.pose_valid);
    Serial.print(" color=");
    Serial.print(incomingCameraPose.color);
    Serial.print(" conf=");
    Serial.print(incomingCameraPose.confidence);
    Serial.print(" tz=");
    Serial.print(incomingCameraPose.tz_mm);
    Serial.println("==============================");

    cameraGuidedPickup();
    return;
  }

  memcpy(&incomingMessage,
         incomingData,
         sizeof(incomingMessage));

  const char* cmd = incomingMessage.command;

  Serial.println("\n==============================");
  Serial.print("ESP-NOW COMMAND RECEIVED: ");
  Serial.println(cmd);
  Serial.println("==============================");

  if (strcmp(cmd, "GTC") == 0)
    GreenToCar();
  else if (strcmp(cmd, "BTC") == 0) {
    BlueToCar();
    Serial.print("BTC");
  }
  else if (strcmp(cmd, "RTC") == 0) {
    RedToCar();
    Serial.print("RTC");
  }
  else if (strcmp(cmd, "BTF") == 0) {
    BlueToFloor();
    Serial.print("BTF");
  }
  else if (strcmp(cmd, "RTF") == 0) {
    RedToFloor();
    Serial.print("RTF");
  }
  else if (strcmp(cmd, "GTF") == 0) {
    GreenToFloor();
    Serial.print("GTF");
  }
  else if (strcmp(cmd, "H") == 0) {
    goHome();
    Serial.print("H");
  }
  else if (strcmp(cmd, "S") == 0) {
    scanPose();
    Serial.print("S");
  }
  else if (strcmp(cmd, "FCTR") == 0) {
    FromCarToRod();
    Serial.print("FCTR");
  }
  else
    Serial.println("[ERROR] Unknown Command");
}

// ================================================================
// SETUP
// ================================================================

void setup() {

  Serial.begin(115200);

  Wire.begin(21, 22);

  driver.begin();

  driver.setOscillatorFrequency(27000000);

  driver.setPWMFreq(SERVO_FREQ);

  for(int i = 0; i < 5; i++) {

    driver.setPWM(i,
                  0,
                  angleToPulse(currentAngle[i]));
  }

  delay(1000);

  // ================= WIFI =================
  WiFi.mode(WIFI_STA);

  WiFi.begin(ssid, pass);

  Serial.print("Connecting WiFi");

  while (WiFi.status() != WL_CONNECTED) {

    delay(500);

    Serial.print(".");
  }

  Serial.println("\nWiFi Connected");

  Serial.print("ARM MAC: ");
  Serial.println(WiFi.macAddress());

  Serial.print("ARM CHANNEL: ");
  Serial.println(WiFi.channel());

  // ================= ESP NOW =================
  if (esp_now_init() != ESP_OK) {

    Serial.println("ESP-NOW INIT FAILED");

    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};

  memcpy(peerInfo.peer_addr, baseAddress, 6);

  peerInfo.channel = WiFi.channel();

  peerInfo.encrypt = false;

  peerInfo.ifidx = WIFI_IF_STA;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {

    Serial.println("FAILED TO ADD BASE PEER");

    return;
  }

  Serial.println("ESP-NOW RECEIVER READY");

  goHome();

  Serial.println("\n--- ARM SYSTEM ONLINE ---");
}

// ================================================================
// LOOP
// ================================================================

void loop() {

}
