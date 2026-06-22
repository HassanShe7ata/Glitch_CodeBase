#include <Wire.h>

// --- I2C Addresses ---
#define MOTOR_ADDR         0x34
#define REG_MOTOR_TYPE     0x14
#define REG_MOTOR_PHASE    0x15
#define REG_FIXED_PWM      0x1F  // CORRECTED: Based on datasheet, open-loop PWM is 0x1F
#define REG_ENCODER_TOTAL  0x3C 
#define SDA_PIN 21
#define SCL_PIN 22

// ================================================================
// SYSTEM IDENTIFICATION CONFIGURATION
// ================================================================
// CHANGE THIS FOR EACH TEST: 
// 0 = Front Left (FL) | 1 = Front Right (FR) | 2 = Back Left (BL) | 3 = Back Right (BR)
const uint8_t TARGET_MOTOR = 0; 

// PRBS Parameters
const unsigned long TEST_DURATION_MS = 15000; // Total test duration (15 seconds)
const unsigned long SAMPLE_TIME_MS   = 10;    // Telemetry sample rate (10ms / 100Hz)
const unsigned long PRBS_CLOCK_MS    = 120;   // How often the PRBS flips state

// Raw PWM scales from -100 to 100 on Hiwonder open-loop registers
const int8_t PRBS_BASE_PWM = 35;  // Baseline cruising duty cycle %
const int8_t PRBS_AMPLITUDE = 15; // Fluctuation range (+/- 15% duty)

// Global Tracking Variables
unsigned long testStartTime = 0;
unsigned long lastSampleTime = 0;
unsigned long lastPrbsFlipTime = 0;
int8_t currentPwmOutput = PRBS_BASE_PWM;

// ================================================================
// I2C MOTOR INTERFACE
// ================================================================

void writeSingleMotorPWM(uint8_t motorIdx, int8_t pwmValue) {
    int8_t pwms[4] = {0, 0, 0, 0};
    pwms[motorIdx] = pwmValue; // Only apply duty cycle to the motor under test
    
    Wire.beginTransmission(MOTOR_ADDR);
    Wire.write(REG_FIXED_PWM); // Using 0x1F raw PWM transmission
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

// ================================================================
// SETUP & LOOP
// ================================================================

void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    
    // --- PRESERVED HIWONDER DRIVER INITIALIZATION ---
    Wire.beginTransmission(MOTOR_ADDR);
    Wire.write(REG_MOTOR_TYPE); Wire.write(3); 
    Wire.endTransmission();
    
    Wire.beginTransmission(MOTOR_ADDR);
    Wire.write(REG_MOTOR_PHASE); Wire.write(0);
    Wire.endTransmission();

    Wire.beginTransmission(MOTOR_ADDR);
    Wire.write(REG_ENCODER_TOTAL);
    for(int i=0; i<16; i++) Wire.write(0);
    Wire.endTransmission();
    
    forceStop();
    
    // Countdown to allow safe deployment onto the floor surface
    Serial.println("--- OPEN-LOOP PRBS DATA CAPTURE PREPARATION ---");
    Serial.print("Targeting Motor Index: "); Serial.println(TARGET_MOTOR);
    for(int i = 5; i > 0; i--) {
        Serial.print("Starting in: "); Serial.println(i);
        delay(1000);
    }
    
    // Print CSV Header
    Serial.println("Timestamp_ms,Input_PWM,Encoder_Ticks");
    
    testStartTime = millis();
    lastSampleTime = millis();
    lastPrbsFlipTime = millis();
}

void loop() {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - testStartTime;

    // Terminate test execution after specified window
    if (elapsedTime >= TEST_DURATION_MS) {
        forceStop();
        Serial.println("--- TEST COMPLETE ---");
        while (true) { delay(1000); } // Idle indefinitely 
    }

    // --- PRBS GENERATOR ENGINE ---
    if (currentTime - lastPrbsFlipTime >= PRBS_CLOCK_MS) {
        lastPrbsFlipTime = currentTime;
        
        long prbsBit = random(0, 2); 
        if (prbsBit == 1) {
            currentPwmOutput = PRBS_BASE_PWM + PRBS_AMPLITUDE; // 35 + 15 = 50% PWM
        } else {
            currentPwmOutput = PRBS_BASE_PWM - PRBS_AMPLITUDE; // 35 - 15 = 20% PWM
        }
        
        // Command only the explicit motor under evaluation using direct PWM
        writeSingleMotorPWM(TARGET_MOTOR, currentPwmOutput);
    }

    // --- DETERMINISTIC TELEMETRY SAMPLING (100Hz) ---
    if (currentTime - lastSampleTime >= SAMPLE_TIME_MS) {
        lastSampleTime = currentTime;
        
        int32_t encoderReadings[4];
        if (readEncoders(encoderReadings)) {
            Serial.print(elapsedTime);
            Serial.print(",");
            Serial.print(currentPwmOutput);
            Serial.print(",");
            Serial.println(encoderReadings[TARGET_MOTOR]);
        }
    }
}