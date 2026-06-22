# Motor System Identification

PRBS-based frequency response characterization for the Glitch robot's mecanum wheel motors.

## Overview

System identification determines the mathematical relationship (transfer function) between PWM input and motor output (encoder velocity). This enables:

- **Model-based PID tuning** instead of manual trial-and-error
- **Predictive simulation** of robot dynamics before hardware deployment
- **Performance verification** against design specifications
- **Fault detection** by comparing measured vs. expected response

## Theory

### PRBS (Pseudo-Random Binary Sequence)

A PRBS signal excites the system across all frequencies simultaneously, making it ideal for frequency-domain identification. The method works by:

1. Apply a binary signal that toggles between two PWM levels
2. Record encoder response at a fixed sample rate
3. Compute cross-correlation between input and output
4. The cross-correlation approximates the impulse response
5. Transform to frequency domain to obtain the Bode plot

### Why LFSR over random()?

The original PRBS_Testing sketch uses `random(0, 2)` which produces a pseudorandom sequence without guaranteed autocorrelation properties. The enhanced version uses a Linear Feedback Shift Register (LFSR) that produces a deterministic maximum-length sequence with ideal autocorrelation:

| Property | random() | LFSR PRBS-7 |
|----------|----------|-------------|
| Sequence length | Unlimited | 127 bits |
| Autocorrelation | Poor | Delta function |
| Reproducibility | Seed-dependent | Deterministic |
| Frequency content | Non-uniform | Uniform across band |

## Hardware Setup

1. Connect ESP32 to motor driver via I2C (SDA=21, SCL=22)
2. Place robot on flat surface with wheels free to rotate
3. Connect ESP32 to PC via USB for serial data capture
4. Flash the PRBS_Enhanced sketch

## Usage

### 1. Flash Firmware

```bash
# Using PlatformIO (if env:prbs is configured)
pio run -e prbs -t upload

# Or flash directly via Arduino IDE
# Open SystemIdentification/PRBS_Enhanced/PRBS_Enhanced.ino
```

### 2. Capture Data

```bash
# Using the capture script
cd SystemIdentification/analysis
python sysid_capture.py COM15 115200 prbs_motor0.csv

# Or capture manually using serial monitor
# Save output to CSV file
```

### 3. Run Analysis

```bash
# Full analysis with plots
python sysid_analysis.py prbs_motor0.csv

# Generate Bode plots from saved model
python sysid_bode.py prbs_motor0_model.json

# Validate model against data
python sysid_validation.py prbs_motor0.csv prbs_motor0_model.json
```

### 4. Install Dependencies

```bash
pip install -r requirements.txt
```

## Test Modes

### PRBS (Mode 0)
- Best for frequency-domain identification
- Excites all frequencies simultaneously
- Default: PRBS-7 (127-bit sequence)

### Step Response (Mode 1)
- Simple time-domain characterization
- Shows rise time, settling time, overshoot
- Good for quick validation

### Sinusoidal Sweep (Mode 2)
- Sweeps from low to high frequency
- Provides direct Bode plot measurement
- Longer test duration but most accurate

## Output Format

### Serial Output (CSV)
```
Timestamp_ms,Input_PWM,Encoder_Ticks,Encoder_Velocity
0,35,0,0
10,50,3,3
20,20,8,5
...
```

### Model JSON
```json
{
  "transfer_function": {
    "numerator": [0.1, 0.2],
    "denominator": [1.0, -0.8, 0.15],
    "zeros": [-2.0],
    "poles": [0.5, 0.3],
    "stable": true
  },
  "frequency_response": {
    "frequency_hz": [...],
    "magnitude": [...],
    "phase_deg": [...]
  }
}
```

## Configuration

### PRBS Parameters (PRBS_Enhanced.ino)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `TARGET_MOTOR` | 0 | Motor index (0=FL, 1=FR, 2=BL, 3=BR) |
| `TEST_MODE` | 0 | 0=PRBS, 1=Step, 2=Sweep |
| `PRBS_ORDER` | 7 | LFSR order (7, 9, or 15) |
| `PRBS_BASE_PWM` | 35 | Center PWM duty cycle |
| `PRBS_AMPLITUDE` | 15 | Toggle range (±15%) |
| `SAMPLE_TIME_MS` | 10 | Telemetry rate (100 Hz) |
| `TEST_DURATION_MS` | 15000 | Test length per run |
| `REPEAT_COUNT` | 1 | Number of repeated runs |

### Tuning Guide

**If signal is too noisy:**
- Increase `PRBS_BASE_PWM` (operates in more linear region)
- Decrease `PRBS_AMPLITUDE` (smaller perturbation)
- Increase `REPEAT_COUNT` for averaging

**If frequency resolution is insufficient:**
- Increase `PRBS_ORDER` (PRBS-9 for 511-bit sequence)
- Increase `TEST_DURATION_MS`

**If motor doesn't respond:**
- Check I2C wiring (SDA=21, SCL=22)
- Verify motor driver power (12V)
- Check `REG_MOTOR_TYPE` is set to 3

## File Structure

```
SystemIdentification/
├── PRBS_Enhanced/
│   └── PRBS_Enhanced.ino          # Enhanced PRBS firmware
├── analysis/
│   ├── sysid_capture.py           # Serial data capture
│   ├── sysid_analysis.py          # Transfer function estimation
│   ├── sysid_bode.py              # Bode plot generation
│   ├── sysid_validation.py        # Model validation
│   └── requirements.txt           # Python dependencies
└── README.md                      # This file
```

## References

1. Ljung, L. (1999). *System Identification: Theory for the User*. Prentice Hall.
2. Pintelon, R., & Schoukens, J. (2012). *System Identification: A Frequency Domain Approach*. Wiley.
3. Espressif ESP32 Technical Reference Manual.
4. Hiwonder MD02 Motor Driver Datasheet.
