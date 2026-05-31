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

const long moveDuration = 1000;

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

void executeSyncMove(long duration) {

  unsigned long startTime = millis();

  for(int i = 0; i < 5; i++)
    startAngles[i] = currentAngle[i];

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

  float q2 =
    atan2(Zw, Rw) +
    atan2(L4 * sin(q3),
    R_up + L4 * cos(q3));

  float d1 = atan2(L3, L2);

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

    executeSyncMove(moveDuration);

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

  moveRobot(posRod[0], posRod[1], posRod[2] + 40, posRod[3], 0);
  moveRobot(posRod[0], posRod[1], posRod[2], posRod[3], 0);
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

  moveRobot(posRod[0], posRod[1], posRod[2] + 40, posRod[3], 0);
  moveRobot(posRod[0], posRod[1], posRod[2], posRod[3], 0);
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

  moveRobot(posRod[0], posRod[1], posRod[2] + 40, posRod[3], 0);
  moveRobot(posRod[0], posRod[1], posRod[2], posRod[3], 0);
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

// ================================================================
// ESP NOW RECEIVE CALLBACK
// ================================================================

void OnDataRecv(const esp_now_recv_info_t *recvInfo,
                const uint8_t *incomingData,
                int len) {

  memcpy(&incomingMessage,
         incomingData,
         sizeof(incomingMessage));

  String cmd = String(incomingMessage.command);

  Serial.println("\n==============================");
  Serial.print("ESP-NOW COMMAND RECEIVED: ");
  Serial.println(cmd);
  Serial.println("==============================");

  if (cmd == "GTC")
    GreenToCar();

  else if (cmd == "BTC"){
    BlueToCar();
      Serial.print("BTC");
  }

  else if (cmd == "RTC"){

    RedToCar();
    Serial.print("RTC");

  }
  else if (cmd == "BTF"){
    BlueToFloor();
    Serial.print("BTF");

  }

  else if (cmd == "RTF"){
    RedToFloor();
    Serial.print("RTF");

  }

   else if (cmd == "GTF"){
    GreenToFloor();
    Serial.print("GTF");

  }

  else if (cmd == "H"){
    goHome();
    Serial.print("H");

  }

  else if (cmd == "S"){
    scanPose();
    Serial.print("S");

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