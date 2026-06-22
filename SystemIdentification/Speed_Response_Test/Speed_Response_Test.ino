/*
 * Speed_Response_Test.ino - Speed Verification Test
 * Glitch Robot Project
 *
 * Commands target speed using REG_FIXED_SPEED (closed-loop, driver PID)
 * and measures actual speed from encoder feedback.
 * Produces verification curve: setpoint vs actual speed.
 *
 * Hardware: ESP32 + Hiwonder MD02 Motor Driver (I2C 0x34)
 * Output: CSV over Serial @ 115200 baud
 */

#include <Wire.h>

// ====================== I2C HARDWARE ======================
#define MOTOR_ADDR         0x34
#define REG_MOTOR_TYPE     0x14
#define REG_MOTOR_PHASE    0x15
#define REG_FIXED_SPEED    0x33  // Closed-loop speed control
#define REG_FIXED_PWM      0x1F  // Open-loop PWM (not used here)
#define REG_ENCODER_TOTAL  0x3C
#define SDA_PIN 21
#define SCL_PIN 22

// ====================== PHYSICAL CONSTANTS ======================
const float MM_PER_TICK = (PI * 80.0f) / 1980.0f;  // 0.12693 mm/tick
const float TICKS_PER_METER = 1000.0f / MM_PER_TICK; // ~7878 ticks/m

// ====================== TEST CONFIGURATION ======================
const uint8_t TARGET_MOTOR = 0;  // 0=FL, 1=FR, 2=BL, 3=BR
const unsigned long SAMPLE_TIME_MS = 10;  // 100 Hz telemetry

// Speed setpoints to test (km/hr)
// Note: Hiwonder speed range is -100 to 100 (percentage of max speed)
// We'll test multiple speed percentages and convert to km/hr
const int8_t SPEED_LEVELS[] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
const uint8_t NUM_SPEED_LEVELS = sizeof(SPEED_LEVELS) / sizeof(SPEED_LEVELS[0]);

const unsigned long STEP_DURATION_MS = 3000;  // 3 seconds per speed level
const unsigned long SETTLE_TIME_MS = 500;     // 0.5s settling before measurement

// ====================== GLOBALS ======================
int32_t prevEncoderTicks = 0;
bool encoderInitialized = false;

// ====================== I2C MOTOR INTERFACE ======================

void writeMotorSpeed(uint8_t motorIdx, int8_t speedValue) {
    int8_t speeds[4] = {0, 0, 0, 0};
    speeds[motorIdx] = speedValue;

    Wire.beginTransmission(MOTOR_ADDR);
    Wire.write(REG_FIXED_SPEED);
    for (int i = 0; i < 4; i++) {
        Wire.write(speeds[i]);
    }
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

void forceStop() {
    Wire.beginTransmission(MOTOR_ADDR);
    Wire.write(REG_FIXED_SPEED);
    for (int i = 0; i < 4; i++) Wire.write(0);
    Wire.endTransmission();
}

// ====================== SPEED COMPUTATION ======================

float computeSpeedKmh(int32_t deltaTicks, unsigned long deltaMs) {
    if (deltaMs == 0) return 0.0f;
    float deltaSeconds = (float)deltaMs / 1000.0f;
    float velocityMmPerSec = (float)deltaTicks * MM_PER_TICK / deltaSeconds;
    float velocityKmPerHr = velocityMmPerSec * 0.0036f;  // mm/s to km/hr
    return velocityKmPerHr;
}

// ====================== SETUP & LOOP ======================

void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);

    // Hiwonder motor driver initialization
    Wire.beginTransmission(MOTOR_ADDR);
    Wire.write(REG_MOTOR_TYPE); Wire.write(3);
    Wire.endTransmission();

    Wire.beginTransmission(MOTOR_ADDR);
    Wire.write(REG_MOTOR_PHASE); Wire.write(0);
    Wire.endTransmission();

    // Zero encoders
    Wire.beginTransmission(MOTOR_ADDR);
    Wire.write(REG_ENCODER_TOTAL);
    for (int i = 0; i < 16; i++) Wire.write(0);
    Wire.endTransmission();

    forceStop();

    // Print metadata
    Serial.println("# ============================================");
    Serial.println("# SPEED RESPONSE VERIFICATION TEST");
    Serial.println("# ============================================");
    Serial.print("# Motor: "); Serial.println(TARGET_MOTOR);
    Serial.print("# Speed Levels: ");
    for (int i = 0; i < NUM_SPEED_LEVELS; i++) {
        Serial.print(SPEED_LEVELS[i]);
        if (i < NUM_SPEED_LEVELS - 1) Serial.print(", ");
    }
    Serial.println("%");
    Serial.print("# Step Duration (ms): "); Serial.println(STEP_DURATION_MS);
    Serial.print("# Sample Time (ms): "); Serial.println(SAMPLE_TIME_MS);
    Serial.print("# MM_PER_TICK: "); Serial.println(MM_PER_TICK, 6);
    Serial.println("# ============================================");

    // Countdown
    for (int i = 5; i > 0; i--) {
        Serial.print("# Starting in: "); Serial.println(i);
        delay(1000);
    }

    // CSV header
    Serial.println("Timestamp_ms,Setpoint_Speed_Pct,Setpoint_Speed_Kmh,Actual_Speed_Kmh,Encoder_Ticks");

    // Initialize encoder baseline
    int32_t encoderReadings[4];
    if (readEncoders(encoderReadings)) {
        prevEncoderTicks = encoderReadings[TARGET_MOTOR];
        encoderInitialized = true;
    }
}

void loop() {
    static unsigned long testStartTime = 0;
    static unsigned long lastSampleTime = 0;
    static uint8_t currentLevel = 0;
    static unsigned long levelStartTime = 0;
    static bool testRunning = false;

    unsigned long currentTime = millis();

    // First run initialization
    if (!testRunning) {
        testStartTime = currentTime;
        lastSampleTime = currentTime;
        levelStartTime = currentTime;
        testRunning = true;
        currentLevel = 0;
        encoderInitialized = false;
    }

    unsigned long elapsedTime = currentTime - testStartTime;
    unsigned long levelElapsed = currentTime - levelStartTime;

    // Check if current speed level is complete
    if (levelElapsed >= STEP_DURATION_MS) {
        // Move to next speed level
        currentLevel++;
        levelStartTime = currentTime;

        if (currentLevel >= NUM_SPEED_LEVELS) {
            // All tests complete
            forceStop();
            Serial.println("# ============================================");
            Serial.println("# ALL TESTS COMPLETE");
            Serial.println("# ============================================");
            while (true) { delay(1000); }
        }

        // Reset encoder tracking for new level
        encoderInitialized = false;
        delay(50);  // Brief settling time
    }

    // Command speed (use REG_FIXED_SPEED for closed-loop control)
    int8_t speedCmd = SPEED_LEVELS[currentLevel];
    writeMotorSpeed(TARGET_MOTOR, speedCmd);

    // Telemetry sampling
    if (currentTime - lastSampleTime >= SAMPLE_TIME_MS) {
        lastSampleTime = currentTime;

        int32_t encoderReadings[4];
        if (readEncoders(encoderReadings)) {
            int32_t currentTicks = encoderReadings[TARGET_MOTOR];

            if (!encoderInitialized) {
                prevEncoderTicks = currentTicks;
                encoderInitialized = true;
                return;  // Skip first sample after level change
            }

            int32_t deltaTicks = currentTicks - prevEncoderTicks;
            unsigned long deltaMs = SAMPLE_TIME_MS;
            prevEncoderTicks = currentTicks;

            // Compute actual speed in km/hr
            float actualSpeedKmh = computeSpeedKmh(deltaTicks, deltaMs);

            // Compute setpoint speed in km/hr
            // Assume max speed ~3 km/hr at 100% command
            float setpointSpeedKmh = (float)speedCmd * 0.03f;

            // Output CSV
            Serial.print(elapsedTime);
            Serial.print(",");
            Serial.print(speedCmd);
            Serial.print(",");
            Serial.print(setpointSpeedKmh, 3);
            Serial.print(",");
            Serial.print(actualSpeedKmh, 3);
            Serial.print(",");
            Serial.println(currentTicks);
        }
    }
}
