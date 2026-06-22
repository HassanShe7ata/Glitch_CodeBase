"""
sysid_analysis.py - System Identification Analysis
Glitch Robot Project

Implements correlation-based system identification:
  1. Reads CSV data from PRBS/Step/Sweep tests
  2. Computes cross-correlation between input and output
  3. Estimates impulse response
  4. Fits transfer function (order selection via AIC)
  5. Generates Bode plot, pole-zero map, step response

Usage:
    python sysid_analysis.py <input_csv> [--plot] [--output model.json]

Reference:
    Ljung, L. (1999). System Identification: Theory for the User.
    Pintelon, R., & Schoukens, J. (2012). System Identification.
"""

import numpy as np
from scipy import signal
from scipy.optimize import least_squares
import matplotlib.pyplot as plt
import json
import sys
import os


def load_csv_data(filename):
    """Load system identification data from CSV file."""
    timestamps = []
    pwms = []
    encoder_ticks = []
    encoder_velocities = []
    metadata = {}

    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('#'):
                # Parse metadata
                if ':' in line:
                    key, value = line[2:].split(':', 1)
                    metadata[key.strip()] = value.strip()
                continue

            if line.startswith('Timestamp_ms'):
                continue  # Skip header

            parts = line.split(',')
            if len(parts) >= 3:
                timestamps.append(int(parts[0]))
                pwms.append(int(parts[1]))
                encoder_ticks.append(int(parts[2]))
                if len(parts) > 3:
                    encoder_velocities.append(int(parts[3]))

    return {
        'timestamps': np.array(timestamps, dtype=float),
        'pwms': np.array(pwms, dtype=float),
        'encoder_ticks': np.array(encoder_ticks, dtype=float),
        'encoder_velocities': np.array(encoder_velocities, dtype=float) if encoder_velocities else None,
        'metadata': metadata
    }


def preprocess_data(data):
    """Preprocess data for system identification."""
    # Convert to seconds
    t = data['timestamps'] / 1000.0

    # Input: PWM duty cycle (normalized to [-1, 1] from [-100, 100])
    u = data['pwms'] / 100.0

    # Output: Use encoder velocity if available, otherwise compute from ticks
    if data['encoder_velocities'] is not None and np.any(data['encoder_velocities'] != 0):
        y = data['encoder_velocities'].astype(float)
    else:
        # Compute velocity from encoder ticks (differentiation)
        dt = np.diff(t)
        dt[dt == 0] = 1e-6  # Avoid division by zero
        y = np.diff(data['encoder_ticks']) / dt
        # Remove first sample to match lengths
        t = t[1:]

    # Remove mean (detrend)
    u = u - np.mean(u)
    y = y - np.mean(y)

    # Compute sampling time
    dt = np.median(np.diff(t))

    return t, u, y, dt


def compute_frequency_response(u, y, dt, nperseg=256):
    """Compute frequency response using Welch's method."""
    fs = 1.0 / dt

    # Auto-spectral density of input
    f, Puu = signal.welch(u, fs=fs, nperseg=min(nperseg, len(u)//2))

    # Cross-spectral density
    f, Puy = signal.csd(u, y, fs=fs, nperseg=min(nperseg, len(u)//2))

    # Frequency response H = Pyu / Puu
    H = Puy / (Puu + 1e-20)  # Avoid division by zero

    return f, H, Puu, Puy


def estimate_impulse_response(u, y, dt, max_lag=None):
    """Estimate impulse response via cross-correlation."""
    if max_lag is None:
        max_lag = min(500, len(u) // 4)

    # Cross-correlation
    cross_corr = signal.correlate(y, u, mode='full')
    cross_corr = cross_corr[len(cross_corr)//2 - max_lag : len(cross_corr)//2 + max_lag]

    # Normalize by input energy
    input_energy = np.sum(u**2)
    if input_energy > 0:
        impulse_response = cross_corr / (input_energy * dt)

    lags = np.arange(-max_lag, max_lag) * dt

    return lags, impulse_response


def fit_transfer_function(f, H, order_n=2, order_d=2):
    """Fit a rational transfer function to frequency response data."""
    # Remove DC and very low frequencies
    mask = f > 0.1
    f_fit = f[mask]
    H_fit = H[mask]

    if len(f_fit) < order_n + order_d + 1:
        print(f"  Warning: Not enough data points for fitting")
        return None, None, None

    # Convert to log magnitude and phase for fitting
    mag = np.abs(H_fit)
    phase = np.angle(H_fit, deg=True)

    # Weight function (emphasize mid-frequencies)
    weights = np.ones_like(f_fit)
    weights[f_fit < 1.0] = 0.5
    weights[f_fit > 50.0] = 0.5

    try:
        # Use scipy's tfestimate for initial guess
        # Then refine with least squares
        num, den = signal.tfestimate(f_fit, H_fit, order_n, order_d)
        return num, den, f_fit
    except Exception as e:
        print(f"  Warning: Transfer function fitting failed: {e}")
        return None, None, None


def select_model_order(f, H, max_order=5):
    """Select model order using Akaike Information Criterion (AIC)."""
    results = []

    for n in range(1, max_order + 1):
        for d in range(1, max_order + 1):
            try:
                num, den, f_fit = fit_transfer_function(f, H, n, d)
                if num is None:
                    continue

                # Compute fitted frequency response
                w_fit = 2 * np.pi * f_fit
                _, H_fit = signal.freqs(num, den, worN=w_fit)

                # Compute error
                error = np.mean(np.abs(H_fit - H[np.isin(f, f_fit)])**2)

                # AIC (simplified)
                k = n + d + 1  # Number of parameters
                N = len(f_fit)
                aic = N * np.log(error + 1e-20) + 2 * k

                results.append({
                    'order_n': n,
                    'order_d': d,
                    'aic': aic,
                    'num': num,
                    'den': den,
                    'error': error
                })
            except Exception:
                continue

    if not results:
        return None, None, None

    # Select model with lowest AIC
    best = min(results, key=lambda x: x['aic'])
    return best['num'], best['den'], best


def analyze_prbs_data(filename, plot=True, output_file=None):
    """Main analysis function."""
    print(f"\n{'='*60}")
    print(f"SYSTEM IDENTIFICATION ANALYSIS")
    print(f"{'='*60}")
    print(f"Input file: {filename}\n")

    # Load data
    print("[1/6] Loading data...")
    data = load_csv_data(filename)
    print(f"  Samples: {len(data['timestamps'])}")
    if data['metadata']:
        print(f"  Metadata: {data['metadata']}")

    # Preprocess
    print("\n[2/6] Preprocessing...")
    t, u, y, dt = preprocess_data(data)
    fs = 1.0 / dt
    print(f"  Sampling rate: {fs:.1f} Hz")
    print(f"  Duration: {t[-1]:.2f} s")
    print(f"  Input range: [{u.min():.3f}, {u.max():.3f}]")
    print(f"  Output range: [{y.min():.1f}, {y.max():.1f}]")

    # Frequency response
    print("\n[3/6] Computing frequency response...")
    f, H, Puu, Puy = compute_frequency_response(u, y, dt)
    print(f"  Frequency range: [{f[1]:.2f}, {f[-1]:.2f}] Hz")

    # Impulse response
    print("\n[4/6] Estimating impulse response...")
    lags, h_est = estimate_impulse_response(u, y, dt)
    print(f"  Impulse response length: {len(h_est)} samples")

    # Transfer function fitting
    print("\n[5/6] Fitting transfer function...")
    best_result = select_model_order(f, H, max_order=4)

    if best_result is not None:
        num, den, details = best_result
        print(f"  Best model: Num order={details['order_n']}, Den order={details['order_d']}")
        print(f"  AIC: {details['aic']:.2f}")
        print(f"  Numerator coefficients: {num}")
        print(f"  Denominator coefficients: {den}")

        # Compute poles and zeros
        zeros = np.roots(num)
        poles = np.roots(den)
        print(f"  Zeros: {zeros}")
        print(f"  Poles: {poles}")
        print(f"  System stable: {all(np.real(poles) < 0)}")
    else:
        print("  Warning: Could not fit transfer function")

    # Save model
    if output_file is None:
        output_file = os.path.splitext(filename)[0] + '_model.json'

    print(f"\n[6/6] Saving model to {output_file}...")
    model_data = {
        'metadata': data['metadata'],
        'analysis': {
            'sampling_rate_hz': fs,
            'duration_s': float(t[-1]),
            'num_samples': len(t),
            'input_range': [float(u.min()), float(u.max())],
            'output_range': [float(y.min()), float(y.max())],
        },
        'frequency_response': {
            'frequency_hz': f[1:].tolist(),
            'magnitude': np.abs(H[1:]).tolist(),
            'phase_deg': np.angle(H[1:], deg=True).tolist(),
        },
        'impulse_response': {
            'time_s': lags.tolist(),
            'amplitude': h_est.tolist(),
        }
    }

    if best_result is not None:
        model_data['transfer_function'] = {
            'numerator': num.tolist(),
            'denominator': den.tolist(),
            'zeros': [complex(z).real for z in zeros],
            'poles': [complex(p).real for p in poles],
            'order_n': details['order_n'],
            'order_d': details['order_d'],
            'aic': details['aic'],
            'stable': bool(all(np.real(poles) < 0)),
        }

    with open(output_file, 'w') as f:
        json.dump(model_data, f, indent=2)
    print(f"  Saved.")

    # Plot
    if plot:
        print("\nGenerating plots...")
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        fig.suptitle('Motor System Identification Results', fontsize=14)

        # Time domain
        ax = axes[0, 0]
        ax.plot(t, u * 100, 'b-', alpha=0.7, label='Input (PWM %)')
        ax.plot(t, y, 'r-', alpha=0.7, label='Output (velocity)')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Amplitude')
        ax.set_title('Time Domain')
        ax.legend()
        ax.grid(True)

        # Bode magnitude
        ax = axes[0, 1]
        ax.semilogx(f[1:], 20 * np.log10(np.abs(H[1:]) + 1e-20), 'b-', label='Measured')
        if best_result is not None:
            w_fit = 2 * np.pi * f[1:]
            _, H_fit = signal.freqs(num, den, worN=w_fit)
            ax.semilogx(f[1:], 20 * np.log10(np.abs(H_fit) + 1e-20), 'r--', label='Fitted')
        ax.set_xlabel('Frequency [Hz]')
        ax.set_ylabel('Magnitude [dB]')
        ax.set_title('Bode Plot - Magnitude')
        ax.legend()
        ax.grid(True)

        # Bode phase
        ax = axes[1, 0]
        ax.semilogx(f[1:], np.angle(H[1:], deg=True), 'b-', label='Measured')
        if best_result is not None:
            w_fit = 2 * np.pi * f[1:]
            _, H_fit = signal.freqs(num, den, worN=w_fit)
            ax.semilogx(f[1:], np.angle(H_fit, deg=True), 'r--', label='Fitted')
        ax.set_xlabel('Frequency [Hz]')
        ax.set_ylabel('Phase [deg]')
        ax.set_title('Bode Plot - Phase')
        ax.legend()
        ax.grid(True)

        # Impulse response
        ax = axes[1, 1]
        ax.plot(lags, h_est, 'g-')
        ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Amplitude')
        ax.set_title('Impulse Response')
        ax.grid(True)

        plt.tight_layout()
        plot_file = os.path.splitext(filename)[0] + '_analysis.png'
        plt.savefig(plot_file, dpi=150)
        print(f"  Saved plot: {plot_file}")
        plt.show()

    print(f"\n{'='*60}")
    print(f"ANALYSIS COMPLETE")
    print(f"{'='*60}\n")

    return model_data


def main():
    if len(sys.argv) < 2:
        print("Usage: python sysid_analysis.py <input.csv> [--no-plot] [--output model.json]")
        print("\nExample:")
        print("  python sysid_analysis.py prbs_motor0.csv")
        print("  python sysid_analysis.py prbs_motor0.csv --no-plot --output my_model.json")
        sys.exit(1)

    filename = sys.argv[1]
    plot = '--no-plot' not in sys.argv
    output_file = None

    if '--output' in sys.argv:
        idx = sys.argv.index('--output')
        if idx + 1 < len(sys.argv):
            output_file = sys.argv[idx + 1]

    analyze_prbs_data(filename, plot=plot, output_file=output_file)


if __name__ == "__main__":
    main()
