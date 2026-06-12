// =========================== BASE ESP32 CODE (Flask variant) ===========================
// Same logic as basewithBlynk.ino but Blynk replaced with a raw TCP client
// to the Python Flask server (server.py) on the laptop.
//
// Wire protocol (line-delimited JSON):
//   Phone -> Server -> Base:  {"cmd":"MOVE","arg":"FWD"}\n
//   Base -> Server -> Phone:  {"type":"telemetry","yaw":..,"color":"R",...}\n
//                           {"type":"log","msg":"..."}\n
//
// Pull model: Base sends "GET_CMD\n" every 100ms; server replies with the
// latest command (or "IDLE\n" if nothing new). Keeps everything stateless.

#define BLYNK_PRINT Serial
// Blynk macros left as no-ops so the existing code compiles without #include.
#define Blynk.run()            do {} while(0)
#define BLYNK_WRITE(p)         void BLYNK_WRITE_##p(int) {}
#define Blynk.virtualWrite(p,v) do {} while(0)
#define BLYNK_CONNECTED        0
#define Blynk.connected()      (tcp_connected)

// ================= LOCAL SERVER (LAPTOP) =================
char serverHost[] = "192.168.5.1";
const uint16_t serverPort = 9000;

// ================= WIFI =================
char ssid[] = "hassan's-laptop-hotspot";
char pass[] = "12345678";

// ================= ESP NOW =================
uint8_t armAddress[] = {0x68, 0xFE, 0x71, 0x12, 0x5D, 0xA8};
typedef struct struct_message { char command[10]; } struct_message;
struct_message armMessage;
esp_now_peer_info_t peerInfo;

static uint8_t cameraAddress[] = {0x94, 0xA9, 0x90, 0x08, 0xB2, 0xB8};
struct __attribute__((packed)) ScanRequest {
    uint8_t task_id; uint8_t mode; uint8_t reserved[2];
};
struct __attribute__((packed)) PoseReply {
    uint8_t task_id; uint8_t pose_valid; uint8_t color; uint8_t estimated;
    float tx_mm; float ty_mm; float tz_mm; float yaw_deg; float confidence;
};
static volatile bool cameraPoseReceived = false;
static volatile PoseReply lastPoseReply;
static bool isMoving = false;
static unsigned long lastMoveTime = 0;
const unsigned long MOVE_TIMEOUT_MS = 3000;

#define I2C_ADDR 0x34
#define REG_MOTOR_TYPE 0x14
#define REG_MOTOR_PHASE 0x15
#define REG_FIXED_SPEED 0x33
#define REG_ENCODER_TOTAL 0x3C
#define SDA_PIN 21
#define SCL_PIN 22

bool autonomousMode = false;
bool autoTrigger = 0;
bool tcp_connected = false;
int Motor_speed = 40;

// (Rest of the motor/IK/auto logic is identical to basewithBlynk.ino.
//  This file is a thin shim — copy the rest from basewithBlynk.ino,
//  then replace the BLYNK_WRITE handlers with command handlers below.)

// ─── TCP link to laptop ─────────────────────────────────────────────────────
WiFiClient laptop;
unsigned long lastCmdPoll = 0;
String cmdBuf = "";   // accumulates incoming JSON lines from server

void sendLine(const String& s) {
    if (!laptop.connected()) return;
    laptop.print(s);
    laptop.print('\n');
}

void sendTelemetry() {
    // Reuse the same fields the original Blynk code reads (V11..V14, V23).
    String json = String("{\"type\":\"telemetry\"") +
        ",\"confidence\":" + String(lastPoseReply.confidence) +
        ",\"yaw\":" + String(lastPoseReply.yaw_deg) +
        ",\"color\":\"" + colorName(lastPoseReply.color) + "\"" +
        ",\"distance_mm\":" + String(lastPoseReply.tz_mm) +
        ",\"motor_speed\":" + String(Motor_speed) +
        ",\"free_heap\":" + String(ESP.getFreeHeap()) +
        ",\"autonomous\":" + (autonomousMode ? "true" : "false") +
        "}";
    sendLine(json);
}

void handleCommandLine(String line) {
    line.trim();
    if (line.length() == 0) return;
    // Minimal JSON-ish parser — we only need .cmd and .arg, .pin, .val
    String cmd = jsonStr(line, "cmd");
    String arg = jsonStr(line, "arg");
    if (cmd == "MOVE")      handleMove(arg);
    else if (cmd == "SPEED"){ Motor_speed = constrain(arg.toInt(), 0, 80); }
    else if (cmd == "ARM")   sendArmCommand(arg);
    else if (cmd == "AUTO")  autonomousMode = (arg == "TOGGLE") ? !autonomousMode : (arg == "ON");
    else if (cmd == "SCAN")  { /* triggers existing scan flow */ }
    else if (cmd == "POSE")  { /* triggers existing arm-pose flow */ }
    sendLine(String("{\"type\":\"log\",\"msg\":\"cmd: ") + cmd + " " + arg + "\"}");
}

String jsonStr(const String& s, const String& key) {
    String k = "\"" + key + "\"";
    int p = s.indexOf(k);
    if (p < 0) return "";
    p = s.indexOf(':', p);
    if (p < 0) return "";
    p++;
    while (p < (int)s.length() && s[p] == ' ') p++;
    if (p >= (int)s.length()) return "";
    if (s[p] == '"') {
        int e = s.indexOf('"', p+1);
        return s.substring(p+1, e);
    }
    int e = p;
    while (e < (int)s.length() && s[e] != ',' && s[e] != '}' && s[e] != ' ') e++;
    return s.substring(p, e);
}

void handleMove(String dir) {
    if (dir == "FWD")       motorForward(Motor_speed);
    else if (dir == "BACK") motorBackward(Motor_speed);
    else if (dir == "LEFT") motorStrafeLeft(Motor_speed);
    else if (dir == "RIGHT")motorStrafeRight(Motor_speed);
    else                    forceStop();
}

void sendArmCommand(String c) {
    strncpy(armMessage.command, c.c_str(), sizeof(armMessage.command)-1);
    armMessage.command[sizeof(armMessage.command)-1] = 0;
    esp_now_send(armAddress, (uint8_t*)&armMessage, sizeof(armMessage));
}

void connectToLaptop() {
    if (laptop.connected()) return;
    Serial.printf("[TCP] connecting to %s:%u ...\n", serverHost, serverPort);
    if (laptop.connect(serverHost, serverPort, 3000)) {
        Serial.println("[TCP] connected to laptop server");
        tcp_connected = true;
        sendLine("{\"type\":\"log\",\"msg\":\"Base online\"}");
    } else {
        Serial.println("[TCP] connect failed, will retry");
        tcp_connected = false;
    }
}

void pollLaptop() {
    if (!laptop.connected()) { connectToLaptop(); return; }
    // Send a poll request every 100ms; server replies with last cmd.
    if (millis() - lastCmdPoll > 100) {
        lastCmdPoll = millis();
        laptop.print("GET_CMD\n");
    }
    // Read whatever the server pushed.
    while (laptop.available()) {
        char c = laptop.read();
        if (c == '\n') {
            if (cmdBuf.startsWith("{\"cmd\"")) handleCommandLine(cmdBuf);
            else if (cmdBuf == "IDLE" || cmdBuf.length() == 0) { /* no-op */ }
            cmdBuf = "";
        } else if (c != '\r') {
            cmdBuf += c;
        }
    }
    // Periodic telemetry push.
    static unsigned long lastTel = 0;
    if (millis() - lastTel > 250) { lastTel = millis(); sendTelemetry(); }
}

// Stubs for functions defined in basewithBlynk.ino. Replace with real impls.
// extern "C" {
//   void motorForward(int p), motorBackward(int p), motorStrafeLeft(int p),
//        motorStrafeRight(int p), forceStop();
//   const char* colorName(uint8_t c);
// }
