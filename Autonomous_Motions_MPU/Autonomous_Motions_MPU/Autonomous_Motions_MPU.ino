#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"

// --- I2C Addresses ---
#define MOTOR_ADDR         0x34
#define REG_MOTOR_TYPE     0x14
#define REG_MOTOR_PHASE    0x15
#define REG_FIXED_SPEED    0x33
#define REG_ENCODER_TOTAL  0x3C 
#define SDA_PIN 21
#define SCL_PIN 22

// --- MPU6050 Objects ---
MPU6050 mpu;
bool dmpReady = false;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
float currentYaw = 0;

// --- Physical Constants (Calibrated) ---
const float TICKS_FWD_BWD = 7504.0; 
const float TICKS_STRAFE  = 8250.0;
const float TICKS_DIAG    = 9450.0; // UPDATED: Fixes 10cm overshoot
const float TICKS_ROTATE  = 7200.0; 

// --- Control Parameters ---
const float KP_POS  = 0.005;          
const float KP_GYRO = 2.5;           
const int8_t MIN_TORQUE = 18;        
const int16_t FINAL_TOLERANCE = 100; 

// --- VECTORS (M1:FL, M2:FR, M3:BL, M4:BR | M1&M4 Inverted) ---
const int8_t V_FORWARD[]    = { 1, -1, -1,  1};
const int8_t V_BACKWARD[]   = {-1,  1,  1, -1};
const int8_t V_STRAFE_L[]   = { 1,  1,  1,  1}; 
const int8_t V_STRAFE_R[]   = {-1, -1, -1, -1}; 
const int8_t V_ROTATE_CW[]  = { 1,  1, -1, -1};
const int8_t V_ROTATE_CCW[] = {-1, -1,  1,  1};

// --- CORRECTED DIAGONALS (Direction Swapped for Hardware) ---
const int8_t V_DIAG_FR[]    = { 1,  0,  0,  1}; // M1(FL) & M4(BR) Active
const int8_t V_DIAG_FL[]    = { 0, -1, -1,  0}; // M2(FR) & M3(BL) Active
const int8_t V_DIAG_BR[]    = { 0,  1,  1,  0}; // M2(FR) & M3(BL) Active
const int8_t V_DIAG_BL[]    = {-1,  0,  0, -1}; // M1(FL) & M4(BR) Active

// ================================================================
// I2C MOTOR INTERFACE
// ================================================================

void writeSpeeds(int8_t fl, int8_t fr, int8_t bl, int8_t br) {
    int8_t speeds[4] = {fl, fr, bl, br};
    Wire.beginTransmission(MOTOR_ADDR);
    Wire.write(REG_FIXED_SPEED);
    for (int i = 0; i < 4; i++) Wire.write(speeds[i]);
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
    for (int i = 0; i < 5; i++) {
        writeSpeeds(0, 0, 0, 0);
        delay(10);
    }
}

// ================================================================
// MPU6050 DMP LOGIC
// ================================================================

void updateYaw() {
    if (dmpReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        currentYaw = (ypr[0] * 180.0f / M_PI);
    }
}

// ================================================================
// NAVIGATION ENGINE
// ================================================================

void moveDistanceKp(const int8_t vector[], int8_t maxSpeed, float distance, float tickConstant) {
    int32_t startEncoders[4], currentEncoders[4];
    long targetTicks = (long)abs(distance * tickConstant);
    
    updateYaw();
    float targetHeading = currentYaw; 
    
    if (!readEncoders(startEncoders)) return;

    long error = targetTicks;
    uint8_t loopCounter = 0;

    while (true) {
        updateYaw();
        
        if (loopCounter % 2 == 0) {
            if (readEncoders(currentEncoders)) {
                double totalTraveled = 0;
                int activeMotors = 0;
                for(int i = 0; i < 4; i++) {
                    if (vector[i] != 0) {
                        totalTraveled += abs((long)currentEncoders[i] - (long)startEncoders[i]);
                        activeMotors++;
                    }
                }
                long traveled = (long)(totalTraveled / (double)activeMotors);
                if (traveled >= targetTicks) break; 
                error = targetTicks - traveled;
            }
        }
        loopCounter++;
        if (error < FINAL_TOLERANCE) break;

        float calcSpeed = (float)error * KP_POS;
        int8_t baseSpeed = (int8_t)constrain(calcSpeed, MIN_TORQUE, maxSpeed);

        float yawError = targetHeading - currentYaw;
        if (yawError > 180) yawError -= 360;
        if (yawError < -180) yawError += 360;
        int8_t gyroCorr = (int8_t)(yawError * KP_GYRO);

        writeSpeeds((baseSpeed * vector[0]) + gyroCorr, 
                    (baseSpeed * vector[1]) - gyroCorr, 
                    (baseSpeed * vector[2]) + gyroCorr, 
                    (baseSpeed * vector[3]) - gyroCorr);
        
        delay(15);
    }
    forceStop();
}

void rotateDegrees(bool clockwise, float degrees, int8_t speed) {
    updateYaw();
    float startHeading = currentYaw;
    float targetHeading = clockwise ? (startHeading + degrees) : (startHeading - degrees);

    while (true) {
        updateYaw();
        float yawError = targetHeading - currentYaw;
        if (yawError > 180) yawError -= 360;
        if (yawError < -180) yawError += 360;

        if (abs(yawError) < 1.0) break; 

        int8_t turnSpeed = (abs(yawError) < 20) ? 20 : speed; 
        if (yawError < 0) turnSpeed = -turnSpeed;
        
        writeSpeeds(turnSpeed * V_ROTATE_CW[0], turnSpeed * V_ROTATE_CW[1], 
                    turnSpeed * V_ROTATE_CW[2], turnSpeed * V_ROTATE_CW[3]);
        delay(10);
    }
    forceStop();
}

// ================================================================
// SETUP & LOOP
// ================================================================

void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    
    mpu.initialize();
    if (mpu.dmpInitialize() == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);
        dmpReady = true;
    }

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
    Serial.println("System Ready.");
}

void loop() {
    // 1m Diagonal Forward-Right
    moveDistanceKp(V_DIAG_FR, 30, 1.0, TICKS_DIAG);
    delay(4000);
    
    // 1m Diagonal Backward-Left
    moveDistanceKp(V_DIAG_BL, 30, 1.0, TICKS_DIAG);
    delay(5000);
}