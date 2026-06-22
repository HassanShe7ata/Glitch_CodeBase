/*
 * Speed_Verification.ino - Forward Speed Response Test
 * Glitch Robot Project
 *
 * Uses EXACT hardware config from base.cpp:
 *   - I2C 0x34, REG_FIXED_SPEED (0x33) for closed-loop speed
 *   - Mecanum forward vector: {+1, -1, -1, +1}
 *   - Encoder constants: 1980 ticks/rev, 80mm wheel
 *   - Speed computation: EMA filtered, mm/s -> km/hr
 *
 * Commands each speed level for 3s, records encoder response.
 * Output: CSV at 115200 baud
 */

#include <Wire.h>

// ==================== HARDWARE CONFIG (from base.cpp) ====================
#define MOTOR_ADDR         0x34
#define REG_MOTOR_TYPE     0x14
#define REG_MOTOR_PHASE    0x15
#define REG_FIXED_SPEED    0x33
#define REG_FIXED_PWM      0x1F
#define REG_ENCODER_TOTAL  0x3C
#define SDA_PIN 21
#define SCL_PIN 22

// ==================== PHYSICAL CONSTANTS (from base.cpp:49-55) ====================
const float TICKS_PER_REV = 1980.0f;
const float WHEEL_DIA_MM  = 80.0f;
const float MM_PER_TICK   = (PI * WHEEL_DIA_MM) / TICKS_PER_REV;  // ~0.1269 mm/tick
const float EMA_ALPHA     = 0.15f;

// ==================== MECANUM FORWARD VECTOR (from base.cpp:260) ====================
// V_FORWARD = {+1, -1, -1, +1}
// FL=+speed, FR=-speed, BL=-speed, BR=+speed

// ==================== TEST CONFIG ====================
const uint8_t  SAMPLE_MS        = 20;     // 50 Hz telemetry
const unsigned long LEVEL_MS    = 4000;   // 4 seconds per speed level
const unsigned long SETTLE_MS   = 500;    // 0.5s settle before recording

// Speed levels to test (-100 to +100 range)
const int8_t SPEED_LEVELS[] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
const uint8_t NUM_LEVELS = sizeof(SPEED_LEVELS) / sizeof(SPEED_LEVELS[0]);

// ==================== GLOBALS ====================
float velX = 0, velY = 0;
float currentSpeed = 0;
int32_t lastEnc[4] = {0, 0, 0, 0};
bool encInit = false;
unsigned long lastEncMs = 0;

// ==================== I2C FUNCTIONS (from base.cpp:396-459) ====================

void writeBytes(uint8_t reg, uint8_t *data, size_t len) {
    Wire.beginTransmission(MOTOR_ADDR);
    Wire.write(reg);
    for (size_t i = 0; i < len; i++) Wire.write(data[i]);
    Wire.endTransmission();
}

bool readEncoders(int32_t* data) {
    Wire.beginTransmission(MOTOR_ADDR);
    Wire.write(REG_ENCODER_TOTAL);
    if (Wire.endTransmission() != 0) return false;
    if (Wire.requestFrom((uint8_t)MOTOR_ADDR, (uint8_t)16) != 16) return false;
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

// Forward movement: FL=+spd, FR=-spd, BL=-spd, BR=+spd
void setForwardSpeed(int8_t speed) {
    int8_t fl =  speed;
    int8_t fr = -speed;
    int8_t bl = -speed;
    int8_t br =  speed;
    int8_t speeds[4] = {fl, fr, bl, br};
    writeBytes(REG_FIXED_SPEED, (uint8_t*)speeds, 4);
}

void stopMotors() {
    int8_t speeds[4] = {0, 0, 0, 0};
    writeBytes(REG_FIXED_SPEED, (uint8_t*)speeds, 4);
}

// ==================== SPEED COMPUTATION (from base.cpp:1153-1174) ====================

float computeSpeed() {
    unsigned long now = millis();
    float dt = (now - lastEncMs) / 1000.0f;
    if (dt < 0.005f) return currentSpeed;
    lastEncMs = now;

    int32_t enc[4];
    if (!readEncoders(enc)) return currentSpeed;

    if (!encInit) {
        for (int i = 0; i < 4; i++) lastEnc[i] = enc[i];
        encInit = true;
        return 0;
    }

    float d[4];
    for (int i = 0; i < 4; i++) {
        d[i] = (float)(enc[i] - lastEnc[i]);
        lastEnc[i] = enc[i];
    }

    // Mecanum inverse kinematics (base.cpp:1162-1163)
    float vx_r = (d[0] - d[1] - d[2] + d[3]) / 4.0f * MM_PER_TICK;
    float vy_r = -(d[0] + d[1] + d[2] + d[3]) / 4.0f * MM_PER_TICK;

    float rawVx = vx_r / dt;
    float rawVy = vy_r / dt;
    float rawSpeed = sqrtf(rawVx * rawVx + rawVy * rawVy);

    // EMA filter (base.cpp:1170-1173)
    velX = EMA_ALPHA * rawVx + (1.0f - EMA_ALPHA) * velX;
    velY = EMA_ALPHA * rawVy + (1.0f - EMA_ALPHA) * velY;
    currentSpeed = EMA_ALPHA * rawSpeed + (1.0f - EMA_ALPHA) * currentSpeed;

    return currentSpeed;  // mm/s
}

// ==================== SETUP & LOOP ====================

void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);

    // Motor driver init (base.cpp pattern)
    uint8_t mtype = 3;
    writeBytes(REG_MOTOR_TYPE, &mtype, 1);
    uint8_t mphase = 0;
    writeBytes(REG_MOTOR_PHASE, &mphase, 1);

    // Zero encoders
    uint8_t zero16[16] = {0};
    writeBytes(REG_ENCODER_TOTAL, zero16, 16);

    stopMotors();

    Serial.println("# SPEED VERIFICATION TEST");
    Serial.println("# Motor: FL(0), Forward vector: {+1,-1,-1,+1}");
    Serial.println("# Using REG_FIXED_SPEED (0x33) - closed-loop driver PID");
    Serial.print("# MM_PER_TICK: "); Serial.println(MM_PER_TICK, 6);

    for (int i = 5; i > 0; i--) {
        Serial.print("# Start in "); Serial.println(i);
        delay(1000);
    }

    Serial.println("Time_ms,Speed_Cmd,Speed_Pct,Actual_mm_s,Actual_Kmh,Enc_FL");

    lastEncMs = millis();
}

void loop() {
    static uint8_t level = 0;
    static unsigned long levelStart = 0;
    static unsigned long sampleLast = 0;
    static bool started = false;

    unsigned long now = millis();

    if (!started) {
        levelStart = now;
        started = true;
    }

    unsigned long levelElapsed = now - levelStart;

    // Level timeout
    if (levelElapsed >= LEVEL_MS) {
        level++;
        levelStart = now;
        encInit = false;  // Reset encoder tracking
        if (level >= NUM_LEVELS) {
            stopMotors();
            Serial.println("# DONE");
            while (true) delay(1000);
        }
        delay(50);  // Brief settle
    }

    // Command speed
    setForwardSpeed(SPEED_LEVELS[level]);

    // Sample
    if (now - sampleLast >= SAMPLE_MS) {
        sampleLast = now;
        float speed_mm_s = computeSpeed();
        float speed_kmh = speed_mm_s * 0.0036f;

        int32_t enc[4];
        readEncoders(enc);

        Serial.print(now);
        Serial.print(",");
        Serial.print(SPEED_LEVELS[level]);
        Serial.print(",");
        Serial.print((int)SPEED_LEVELS[level]);
        Serial.print(",");
        Serial.print(speed_mm_s, 1);
        Serial.print(",");
        Serial.print(speed_kmh, 3);
        Serial.print(",");
        Serial.println(enc[0]);
    }
}
