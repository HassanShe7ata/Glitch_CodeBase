# System Identification Methodology

## Academic/Industrial Documentation

**Version:** 1.0  
**Date:** 2026  
**Project:** Glitch - Omnidirectional Mecanum Wheel Robot with Computer Vision

---

## Table of Contents

1. [Overview](#overview)
2. [Theoretical Background](#theoretical-background)
3. [PRBS Signal Generation](#prbs-signal-generation)
4. [Experimental Setup](#experimental-setup)
5. [Data Acquisition](#data-acquisition)
6. [Frequency Response Estimation](#frequency-response-estimation)
7. [Transfer Function Fitting](#transfer-function-fitting)
8. [Model Validation](#model-validation)
9. [Practical Considerations](#practical-considerations)
10. [Controller Design Implications](#controller-design-implications)

---

## Overview

System identification determines the mathematical relationship between a system's input (PWM duty cycle) and output (encoder velocity/ticks). For the Glitch robot's mecanum wheel motors, this enables:

- **Model-based PID tuning** replacing manual trial-and-error
- **Predictive simulation** of robot dynamics before hardware testing
- **Performance verification** against design specifications
- **Fault detection** by comparing measured vs. expected response

### Why System Identification?

The Hiwonder MD02 motor driver's internal PID controller is not tunable via software. The robot's base controller applies open-loop PWM (register 0x1F) and reads encoder feedback (register 0x3C). To design an effective external controller, we must characterize the motor's dynamic response.

### What We Identify

The plant model relates PWM input to encoder velocity output:

```
          G(s)
u(t) ────────────► y(t)
  PWM              Velocity
  (duty%)          (ticks/sample)
```

The transfer function G(s) captures:
- Motor electrical dynamics (inductance, resistance)
- Mechanical dynamics (inertia, friction)
- Encoder quantization effects
- Driver response characteristics

---

## Theoretical Background

### Frequency Domain Analysis

A linear time-invariant (LTI) system can be characterized by its frequency response H(jω):

```
H(jω) = Y(jω) / U(jω)
```

where:
- U(jω) is the Fourier transform of the input signal
- Y(jω) is the Fourier transform of the output signal
- H(jω) is complex-valued with magnitude |H(jω)| and phase ∠H(jω)

### Cross-Correlation Method

For PRBS excitation, the impulse response h(τ) can be estimated via cross-correlation:

```
h(τ) ≈ Ruy(τ) / Ruu(0)
```

where:
- Ruy(τ) = E[u(t) · y(t+τ)] is the cross-correlation
- Ruu(0) = E[u(t)²] is the input energy

The frequency response is then obtained by Fourier transform of the impulse response estimate.

### Why PRBS?

PRBS excitation offers several advantages over other identification methods:

| Method | Pros | Cons |
|--------|------|------|
| **Step Response** | Simple, intuitive | Limited frequency content |
| **Sinusoidal Sweep** | Accurate, direct Bode plot | Slow, requires precise frequency control |
| **White Noise** | Uniform frequency content | Infinite duration, impractical |
| **PRBS** | Finite duration, uniform spectrum, reproducible | Requires LFSR implementation |

---

## PRBS Signal Generation

### Linear Feedback Shift Register (LFSR)

A PRBS is generated using a maximal-length LFSR. For order N, the sequence has length 2^N - 1 bits.

#### PRBS-7 (Order 7)
- Polynomial: x^7 + x^6 + 1
- Sequence length: 127 bits
- Bit period: 70ms (configurable)

```cpp
// LFSR feedback for PRBS-7
feedback = ((lfsrState >> 6) ^ (lfsrState >> 5)) & 1;
lfsrState = (lfsrState << 1) | feedback;
```

#### PRBS-9 (Order 9)
- Polynomial: x^9 + x^5 + 1
- Sequence length: 511 bits
- Better frequency resolution than PRBS-7

#### PRBS-15 (Order 15)
- Polynomial: x^15 + x^14 + 1
- Sequence length: 32767 bits
- Highest frequency resolution, longest test duration

### PWM Mapping

The PRBS binary output (0/1) is mapped to PWM levels:

```
PRBS bit = 1 → PWM = BASE + AMPLITUDE (e.g., 35 + 15 = 50%)
PRBS bit = 0 → PWM = BASE - AMPLITUDE (e.g., 35 - 15 = 20%)
```

The BASE PWM ensures the motor operates in a linear region above the deadband. The AMPLITUDE determines the signal-to-noise ratio of the excitation.

### Autocorrelation Properties

A maximal-length PRBS has autocorrelation:

```
Ruu(τ) = { 1,           τ = 0
         { -1/(2^N-1),  τ ≠ 0
```

This approaches a delta function for large N, making the cross-correlation a good impulse response estimate.

---

## Experimental Setup

### Hardware Configuration

```
ESP32 (I2C Master)
    │
    ├── SDA (GPIO 21) ──┐
    │                    │
    ├── SCL (GPIO 22) ──┤
    │                    │
    └── 5V / GND ───────┤
                         │
                    ┌────┴────┐
                    │ Hiwonder │
                    │ MD02     │
                    │ (0x34)   │
                    └────┬────┘
                         │
            ┌────────────┼────────────┐
            │            │            │
         Motor FL     Motor FR     Motor BL / BR
         (Target)     (Idle)       (Idle)
```

### Test Conditions

| Parameter | Value | Notes |
|-----------|-------|-------|
| Surface | Flat, clean | Consistent friction |
| Load | Robot weight only | No additional payload |
| Temperature | Room temperature | Affects motor resistance |
| Battery | Fully charged | Stable voltage supply |
| Duration | 15 seconds | Sufficient for PRBS-7 |

### Safety Precautions

1. **5-second countdown** before test begins (allows safe deployment)
2. **Emergency stop** via `forceStop()` on test completion
3. **Single motor excitation** (other motors remain idle)
4. **Encoder monitoring** for stall detection

---

## Data Acquisition

### Serial Output Format

The firmware outputs CSV data at 115200 baud:

```csv
# Metadata header lines (starting with #)
Timestamp_ms,Input_PWM,Encoder_Ticks,Encoder_Velocity
0,35,0,0
10,50,3,3
20,20,8,5
30,50,15,7
...
```

### Data Fields

| Field | Type | Units | Description |
|-------|------|-------|-------------|
| `Timestamp_ms` | uint32 | ms | Elapsed time since test start |
| `Input_PWM` | int8 | % | PWM duty cycle (-100 to 100) |
| `Encoder_Ticks` | int32 | ticks | Absolute encoder count |
| `Encoder_Velocity` | int32 | ticks/sample | Differential encoder velocity |

### Sample Rate

- **Telemetry rate:** 100 Hz (10ms sample period)
- **PRBS clock:** Configurable (default: 70ms for PRBS-7)
- **Nyquist frequency:** 50 Hz

### Data Capture Methods

#### Using sysid_capture.py

```bash
cd SystemIdentification/analysis
python sysid_capture.py COM15 115200 prbs_motor0.csv
```

#### Using PlatformIO Monitor

```bash
pio device monitor -p COM15 -b 115200 | tee prbs_data.csv
```

---

## Frequency Response Estimation

### Welch's Method

The frequency response is estimated using Welch's method:

1. **Segment** the data into overlapping windows
2. **Compute** auto-spectral density Puu(f) of input
3. **Compute** cross-spectral density Puy(f) of input-output
4. **Estimate** H(f) = Puy(f) / Puu(f)

### Implementation

```python
from scipy import signal

# Compute frequency response
f, Puu = signal.welch(u, fs=fs, nperseg=256)
f, Puy = signal.csd(u, y, fs=fs, nperseg=256)

# Transfer function estimate
H = Puy / (Puu + 1e-20)  # Avoid division by zero
```

### Bode Plot

The Bode plot displays:
- **Magnitude:** 20·log₁₀(|H(f)|) in dB
- **Phase:** arg(H(f)) in degrees

Key features to identify:
- **DC gain:** Low-frequency magnitude
- **Bandwidth:** -3dB frequency
- **Resonance peak:** Underdamped oscillations
- **Roll-off rate:** High-frequency attenuation (dB/decade)

---

## Transfer Function Fitting

### Model Structure

The motor plant is typically modeled as a second-order system:

```
         K · ωn²
G(s) = ─────────────────
        s² + 2ζωn·s + ωn²
```

where:
- K = DC gain
- ωn = natural frequency (rad/s)
- ζ = damping ratio

### Order Selection

The Akaike Information Criterion (AIC) selects the optimal model order:

```
AIC = N · log(error) + 2k
```

where:
- N = number of data points
- error = mean squared prediction error
- k = number of model parameters

Lower AIC indicates better model with appropriate complexity.

### Fitting Results

Typical motor characteristics:
- **DC gain:** 0.5 - 2.0 (velocity per unit PWM)
- **Natural frequency:** 5 - 20 Hz
- **Damping ratio:** 0.3 - 0.8
- **Settling time:** 0.2 - 1.0 seconds

---

## Model Validation

### Goodness-of-Fit Metrics

| Metric | Formula | Excellent | Good | Fair |
|--------|---------|-----------|------|------|
| R² | 1 - SS_res/SS_tot | > 0.9 | > 0.7 | > 0.5 |
| NRMSE | RMSE/(y_max - y_min) | < 0.1 | < 0.2 | < 0.3 |
| Correlation | corrcoef(y_meas, y_sim) | > 0.95 | > 0.85 | > 0.7 |

### Validation Procedure

1. **Compare** simulated vs. measured step response
2. **Compute** prediction error residuals
3. **Check** residual whiteness (no unmodeled dynamics)
4. **Verify** stability (all poles in left half-plane)

### Common Issues

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| Low R² | Nonlinearities | Use higher PRBS amplitude |
| Oscillatory fit | Underestimated order | Increase model order |
| Poor low-freq fit | Insufficient test duration | Increase TEST_DURATION_MS |
| Noise in Bode plot | Insufficient averaging | Increase REPEAT_COUNT |

---

## Practical Considerations

### PRBS Parameter Tuning

| Parameter | Effect of Increasing | Typical Range |
|-----------|---------------------|---------------|
| `PRBS_ORDER` | Better frequency resolution | 7, 9, 15 |
| `PRBS_BASE_PWM` | More linear operating point | 25 - 50% |
| `PRBS_AMPLITUDE` | Higher SNR, more nonlinear | 10 - 25% |
| `SAMPLE_TIME_MS` | Higher frequency resolution | 5 - 20 ms |
| `TEST_DURATION_MS` | Better low-freq characterization | 10 - 60 s |

### Nonlinear Effects

Real motors exhibit nonlinearities:
- **Deadband:** Minimum PWM to overcome static friction
- **Saturation:** Maximum velocity at high PWM
- **Hysteresis:** Different response for increasing vs. decreasing PWM
- **Cogging:** Periodic torque ripple from motor construction

PRBS identification linearizes around the operating point (BASE PWM). Multiple tests at different BASE PWM values can characterize nonlinear effects.

### Environmental Factors

- **Surface friction:** Affects mechanical load
- **Battery voltage:** Affects motor torque
- **Temperature:** Affects motor resistance and encoder accuracy
- **Wheel slip:** Invalidates encoder-based velocity measurement

---

## Controller Design Implications

### PID Tuning

With the identified transfer function G(s), PID gains can be calculated using:

1. **Ziegler-Nichols:** Based on ultimate gain and period
2. **Internal Model Control (IMC):** Based on time constant and delay
3. **Pole placement:** Direct specification of closed-loop dynamics

### Feedforward Control

The DC gain of G(s) provides the feedforward gain:

```
u_ff = (desired_velocity) / K_dc
```

### State Estimation

The identified model enables Kalman filtering for velocity estimation, combining encoder measurements with the dynamic model for noise reduction.

---

## References

1. Ljung, L. (1999). *System Identification: Theory for the User*. Prentice Hall.
2. Pintelon, R., & Schoukens, J. (2012). *System Identification: A Frequency Domain Approach*. Wiley.
3. Espressif ESP32 Technical Reference Manual.
4. Hiwonder MD02 Motor Driver Datasheet.

---

**End of Document**
