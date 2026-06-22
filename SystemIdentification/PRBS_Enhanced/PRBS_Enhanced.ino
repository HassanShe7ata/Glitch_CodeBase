/*
 * PRBS_Enhanced.ino - Enhanced PRBS Motor System Identification
 * Glitch Robot Project
 *
 * Features over original PRBS_Testing.ino:
 *   - True LFSR-based PRBS (PRBS-7/9/15) with deterministic autocorrelation
 *   - Multi-test modes: PRBS, Step Response, Sinusoidal Sweep
 *   - Encoder velocity computation (differential encoding)
 *   - Configurable amplitude levels for nonlinear characterization
 *   - Repeat count for statistical averaging
 *   - Metadata header in CSV for automated parsing
 *
 * Hardware: ESP32 + Hiwonder MD02 Motor Driver (I2C 0x34)
 * Output: CSV over Serial @ 115200 baud
 */

#include <Wire.h>

// ====================== I2C HARDWARE ======================
#define MOTOR_ADDR         0x34
#define REG_MOTOR_TYPE     0x14
#define REG_MOTOR_PHASE    0x15
#define REG_FIXED_PWM      0x1F
#define REG_ENCODER_TOTAL  0x3C
#define SDA_PIN 21
#define SCL_PIN 22

// ====================== TEST CONFIGURATION ======================
// Motor: 0=FL, 1=FR, 2=BL, 3=BR
const uint8_t TARGET_MOTOR = 0;

// Test mode: 0=PRBS, 1=Step Response, 2=Sinusoidal Sweep
const uint8_t TEST_MODE = 0;

// PRBS order: 7=PRBS-7 (127 bits), 9=PRBS-9 (511 bits), 15=PRBS-15 (32767 bits)
const uint8_t PRBS_ORDER = 7;

// Timing
const unsigned long SAMPLE_TIME_MS = 10;     // 100 Hz telemetry
const unsigned long TEST_DURATION_MS = 15000; // 15 seconds per run
const uint8_t REPEAT_COUNT = 1;               // Number of repeated runs

// PWM parameters (range: -100 to 100)
const int8_t PRBS_BASE_PWM = 35;
const int8_t PRBS_AMPLITUDE = 15;
const int8_t STEP_LOW_PWM = 20;
const int8_t STEP_HIGH_PWM = 50;

// Sinusoidal sweep parameters
const float SWEEP_FREQ_MIN = 0.5f;  // Hz
const float SWEEP_FREQ_MAX = 10.0f; // Hz
const unsigned long SWEEP_DURATION_MS = 10000;

// ====================== LFSR PRBS ENGINE ======================
// Maximum-length LFSR sequences for system identification
// Polynomial taps chosen for maximal length (2^N - 1 bits)

static uint32_t lfsrState = 0x00000001;
static uint8_t lfsrBitCount = 0;

void lfsrInit(uint8_t order) {
    lfsrState = 0x00000001;
    lfsrBitCount = 0;
    // Ensure non-zero initial state
    if (lfsrState == 0) lfsrState = 0x00000001;
}

uint8_t lfsrStep(uint8_t order) {
    uint32_t feedback = 0;
    uint32_t mask = 0;
    switch (order) {
        case 7:  // x^7 + x^6 + 1
            feedback = ((lfsrState >> 6) ^ (lfsrState >> 5)) & 1;
            mask = 0x7F;   // 2^7 - 1
            break;
        case 9:  // x^9 + x^5 + 1
            feedback = ((lfsrState >> 8) ^ (lfsrState >> 4)) & 1;
            mask = 0x1FF;  // 2^9 - 1
            break;
        case 15: // x^15 + x^14 + 1
            feedback = ((lfsrState >> 14) ^ (lfsrState >> 13)) & 1;
            mask = 0x7FFF; // 2^15 - 1
            break;
        default: // Default to PRBS-7
            feedback = ((lfsrState >> 6) ^ (lfsrState >> 5)) & 1;
            mask = 0x7F;
            break;
    }
    lfsrState = ((lfsrState << 1) | feedback) & mask;
    lfsrBitCount++;
    return (uint8_t)(lfsrState & 1);
}

uint8_t getPrbsBit() {
    return lfsrStep(PRBS_ORDER);
}

// ====================== I2C MOTOR INTERFACE ======================

void writeSingleMotorPWM(uint8_t motorIdx, int8_t pwmValue) {
    int8_t pwms[4] = {0, 0, 0, 0};
    pwms[motorIdx] = pwmValue;

    Wire.beginTransmission(MOTOR_ADDR);
    Wire.write(REG_FIXED_PWM);
    for (int i = 0; i < 4; i++) {
        Wire.write(pwms[i]);
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
    Wire.write(REG_FIXED_PWM);
    for (int i = 0; i < 4; i++) Wire.write(0);
    Wire.endTransmission();
}

// ====================== TEST SIGNAL GENERATORS ======================

int8_t generatePrbsSignal(unsigned long elapsedMs) {
    static unsigned long lastFlipTime = 0;
    static int8_t currentPwm = PRBS_BASE_PWM;

    // PRBS clock: 1 bit per sample period for maximum frequency content
    // For PRBS-7 with 10ms sample: full sequence = 127 * 10ms = 1.27s
    unsigned long prbsClockMs = SAMPLE_TIME_MS;

    if (elapsedMs - lastFlipTime >= prbsClockMs) {
        lastFlipTime = elapsedMs;
        uint8_t bit = getPrbsBit();
        currentPwm = bit ? (PRBS_BASE_PWM + PRBS_AMPLITUDE) : (PRBS_BASE_PWM - PRBS_AMPLITUDE);
    }
    return currentPwm;
}

int8_t generateStepSignal(unsigned long elapsedMs) {
    // Step from low to high at t = 1 second
    if (elapsedMs < 1000) return STEP_LOW_PWM;
    return STEP_HIGH_PWM;
}

int8_t generateSweepSignal(unsigned long elapsedMs) {
    // Logarithmic frequency sweep from SWEEP_FREQ_MIN to SWEEP_FREQ_MAX
    float t = (float)elapsedMs / 1000.0f;
    float duration = (float)SWEEP_DURATION_MS / 1000.0f;

    if (t > duration) return PRBS_BASE_PWM;

    // Logarithmic sweep: f(t) = f_min * (f_max/f_min)^(t/T)
    float ratio = SWEEP_FREQ_MAX / SWEEP_FREQ_MIN;
    float freq = SWEEP_FREQ_MIN * pow(ratio, t / duration);

    // Correct phase for chirp: integral of instantaneous frequency
    // phi(t) = 2*pi*f_min*T/ln(ratio) * [ratio^(t/T) - 1]
    float phase = 2.0f * PI * SWEEP_FREQ_MIN * duration / log(ratio) *
                  (pow(ratio, t / duration) - 1.0f);

    float signal = sin(phase);

    // Scale to PWM range
    return (int8_t)(PRBS_BASE_PWM + (int8_t)(signal * PRBS_AMPLITUDE));
}

int8_t generateTestSignal(unsigned long elapsedMs) {
    switch (TEST_MODE) {
        case 0: return generatePrbsSignal(elapsedMs);
        case 1: return generateStepSignal(elapsedMs);
        case 2: return generateSweepSignal(elapsedMs);
        default: return generatePrbsSignal(elapsedMs);
    }
}

// ====================== SETUP & LOOP ======================

static int32_t prevEncoderTicks = 0;
static bool encoderInitialized = false;

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

    Wire.beginTransmission(MOTOR_ADDR);
    Wire.write(REG_ENCODER_TOTAL);
    for (int i = 0; i < 16; i++) Wire.write(0);
    Wire.endTransmission();

    forceStop();

    // Print test metadata header
    Serial.println("# ============================================");
    Serial.println("# SYSTEM IDENTIFICATION TEST");
    Serial.println("# ============================================");
    Serial.print("# Motor: "); Serial.println(TARGET_MOTOR);
    Serial.print("# Mode: ");
    switch (TEST_MODE) {
        case 0: Serial.print("PRBS-"); Serial.println(PRBS_ORDER); break;
        case 1: Serial.println("Step Response"); break;
        case 2: Serial.println("Sinusoidal Sweep"); break;
        default: Serial.println("Unknown"); break;
    }
    Serial.print("# Base PWM: "); Serial.println(PRBS_BASE_PWM);
    Serial.print("# Amplitude: "); Serial.println(PRBS_AMPLITUDE);
    Serial.print("# Sample Time (ms): "); Serial.println(SAMPLE_TIME_MS);
    Serial.print("# Duration (ms): "); Serial.println(TEST_DURATION_MS);
    Serial.print("# Repeat Count: "); Serial.println(REPEAT_COUNT);
    if (TEST_MODE == 0) {
        Serial.print("# PRBS Sequence Length: ");
        switch (PRBS_ORDER) {
            case 7: Serial.println(127); break;
            case 9: Serial.println(511); break;
            case 15: Serial.println(32767); break;
            default: Serial.println(127); break;
        }
    }
    Serial.println("# ============================================");

    // Countdown
    for (int i = 5; i > 0; i--) {
        Serial.print("# Starting in: "); Serial.println(i);
        delay(1000);
    }

    // CSV header
    Serial.println("Timestamp_ms,Input_PWM,Encoder_Ticks,Encoder_Velocity");

    // Initialize encoder baseline
    int32_t encoderReadings[4];
    if (readEncoders(encoderReadings)) {
        prevEncoderTicks = encoderReadings[TARGET_MOTOR];
        encoderInitialized = true;
    }

    // Initialize LFSR for PRBS mode
    lfsrInit(PRBS_ORDER);
}

void loop() {
    static unsigned long testStartTime = 0;
    static unsigned long lastSampleTime = 0;
    static uint8_t currentRepeat = 0;
    static bool testRunning = false;

    unsigned long currentTime = millis();

    // First run initialization
    if (!testRunning) {
        testStartTime = currentTime;
        lastSampleTime = currentTime;
        testRunning = true;
        currentRepeat = 0;
        encoderInitialized = false;
    }

    unsigned long elapsedTime = currentTime - testStartTime;

    // Check if current run is complete
    if (elapsedTime >= TEST_DURATION_MS) {
        forceStop();
        currentRepeat++;
        delay(500); // Brief pause between repeats

        if (currentRepeat >= REPEAT_COUNT) {
            Serial.println("# ============================================");
            Serial.println("# ALL TESTS COMPLETE");
            Serial.println("# ============================================");
            while (true) { delay(1000); }
        }

        // Reset for next repeat
        testStartTime = millis();
        lastSampleTime = testStartTime;
        lfsrInit(PRBS_ORDER);
        encoderInitialized = false;
        delay(200);
    }

    // Generate and apply test signal
    int8_t pwmOutput = generateTestSignal(elapsedTime);
    writeSingleMotorPWM(TARGET_MOTOR, pwmOutput);

    // Telemetry sampling at deterministic rate
    if (currentTime - lastSampleTime >= SAMPLE_TIME_MS) {
        lastSampleTime = currentTime;

        int32_t encoderReadings[4];
        if (readEncoders(encoderReadings)) {
            int32_t currentTicks = encoderReadings[TARGET_MOTOR];

            // Initialize baseline on first valid read
            if (!encoderInitialized) {
                prevEncoderTicks = currentTicks;
                encoderInitialized = true;
            }

            // Compute encoder velocity (ticks per sample period)
            int32_t velocity = currentTicks - prevEncoderTicks;
            prevEncoderTicks = currentTicks;

            // Output CSV: timestamp, pwm, encoder_ticks, encoder_velocity
            Serial.print(elapsedTime);
            Serial.print(",");
            Serial.print(pwmOutput);
            Serial.print(",");
            Serial.print(currentTicks);
            Serial.print(",");
            Serial.println(velocity);
        }
    }
}
