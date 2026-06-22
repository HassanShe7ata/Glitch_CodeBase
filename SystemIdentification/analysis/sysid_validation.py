"""
sysid_validation.py - Model Validation
Glitch Robot Project

Validates identified transfer function against measured data.
Computes fit metrics, generates comparison plots.

Usage:
    python sysid_validation.py <data.csv> <model.json> [--output validation.png]
"""

import numpy as np
from scipy import signal
import matplotlib.pyplot as plt
import json
import sys
import os


def load_data(filename):
    """Load measurement data from CSV."""
    timestamps = []
    pwms = []
    encoder_ticks = []
    encoder_velocities = []

    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('#') or line.startswith('Timestamp'):
                continue

            parts = line.split(',')
            if len(parts) >= 3:
                timestamps.append(int(parts[0]))
                pwms.append(int(parts[1]))
                encoder_ticks.append(int(parts[2]))
                if len(parts) > 3:
                    encoder_velocities.append(int(parts[3]))

    return {
        'timestamps': np.array(timestamps, dtype=float) / 1000.0,
        'pwms': np.array(pwms, dtype=float),
        'encoder_ticks': np.array(encoder_ticks, dtype=float),
        'encoder_velocities': np.array(encoder_velocities, dtype=float) if encoder_velocities else None
    }


def load_model(filename):
    """Load model from JSON."""
    with open(filename, 'r') as f:
        return json.load(f)


def simulate_transfer_function(num, den, u, dt):
    """Simulate system response using identified transfer function."""
    # Create state-space representation for simulation
    sys_tf = signal.TransferFunction(num, den)

    # Discretize for simulation
    dt_sim = dt
    sys_d = signal.cont2discrete((num, den), dt_sim, method='zoh')

    # Simulate
    t_out, y_out, _ = signal.lsim(sys_d, u, np.arange(len(u)) * dt_sim)

    return y_out


def compute_fit_metrics(y_meas, y_sim):
    """Compute goodness-of-fit metrics."""
    # Remove mean for comparison
    y_meas_detrend = y_meas - np.mean(y_meas)
    y_sim_detrend = y_sim - np.mean(y_sim)

    # Normalized Root Mean Square Error (NRMSE)
    rmse = np.sqrt(np.mean((y_meas - y_sim)**2))
    nrmse = rmse / (np.max(y_meas) - np.min(y_meas) + 1e-20)

    # R-squared (coefficient of determination)
    ss_res = np.sum((y_meas - y_sim)**2)
    ss_tot = np.sum((y_meas_detrend)**2)
    r_squared = 1 - (ss_res / (ss_tot + 1e-20))

    # Mean Absolute Error
    mae = np.mean(np.abs(y_meas - y_sim))

    # Maximum Absolute Error
    max_err = np.max(np.abs(y_meas - y_sim))

    # Cross-correlation coefficient
    corr = np.corrcoef(y_meas, y_sim)[0, 1]

    return {
        'rmse': rmse,
        'nrmse': nrmse,
        'r_squared': r_squared,
        'mae': mae,
        'max_error': max_err,
        'correlation': corr
    }


def validate_model(data_file, model_file, output_file=None, show=True):
    """Validate identified model against measurement data."""
    print(f"\n{'='*60}")
    print(f"MODEL VALIDATION")
    print(f"{'='*60}")

    # Load data and model
    print("\n[1/4] Loading data and model...")
    data = load_data(data_file)
    model = load_model(model_file)

    print(f"  Data file: {data_file}")
    print(f"  Model file: {model_file}")

    if 'transfer_function' not in model:
        print("Error: No transfer function in model.")
        return None

    tf = model['transfer_function']
    num = np.array(tf['numerator'])
    den = np.array(tf['denominator'])

    print(f"  Transfer function order: {tf['order_n']}/{tf['order_d']}")

    # Preprocess data
    print("\n[2/4] Preprocessing...")
    t = data['timestamps']
    u = data['pwms'] / 100.0  # Normalize to [-1, 1]
    dt = np.median(np.diff(t))

    if data['encoder_velocities'] is not None and np.any(data['encoder_velocities'] != 0):
        y_meas = data['encoder_velocities'].astype(float)
    else:
        # Compute velocity from ticks
        y_meas = np.diff(data['encoder_ticks']) / dt
        t = t[1:]

    # Remove mean
    u = u - np.mean(u)
    y_meas = y_meas - np.mean(y_meas)

    print(f"  Samples: {len(t)}")
    print(f"  Duration: {t[-1]:.2f} s")

    # Simulate model response
    print("\n[3/4] Simulating model response...")
    y_sim = simulate_transfer_function(num, den, u, dt)

    # Align lengths (simulation may have different length)
    min_len = min(len(y_meas), len(y_sim))
    y_meas = y_meas[:min_len]
    y_sim = y_sim[:min_len]
    t_plot = t[:min_len]

    # Compute fit metrics
    metrics = compute_fit_metrics(y_meas, y_sim)

    print(f"\n  Fit Metrics:")
    print(f"    R-squared:     {metrics['r_squared']:.4f}")
    print(f"    NRMSE:         {metrics['nrmse']:.4f}")
    print(f"    RMSE:          {metrics['rmse']:.2f}")
    print(f"    MAE:           {metrics['mae']:.2f}")
    print(f"    Max Error:     {metrics['max_error']:.2f}")
    print(f"    Correlation:   {metrics['correlation']:.4f}")

    # Quality assessment
    print(f"\n  Model Quality Assessment:")
    if metrics['r_squared'] > 0.9:
        print(f"    [EXCELLENT] R² > 0.9")
    elif metrics['r_squared'] > 0.7:
        print(f"    [GOOD] R² > 0.7")
    elif metrics['r_squared'] > 0.5:
        print(f"    [FAIR] R² > 0.5")
    else:
        print(f"    [POOR] R² < 0.5 - Consider higher order model or different identification method")

    # Save validation results
    if output_file is None:
        output_file = os.path.splitext(data_file)[0] + '_validation.json'

    validation_data = {
        'data_file': data_file,
        'model_file': model_file,
        'transfer_function': tf,
        'metrics': metrics,
        'model_quality': 'EXCELLENT' if metrics['r_squared'] > 0.9 else
                         'GOOD' if metrics['r_squared'] > 0.7 else
                         'FAIR' if metrics['r_squared'] > 0.5 else 'POOR'
    }

    json_file = os.path.splitext(output_file)[0] + '.json'
    with open(json_file, 'w') as f:
        json.dump(validation_data, f, indent=2)
    print(f"\n  Validation results saved: {json_file}")

    # Generate plot
    print("\n[4/4] Generating validation plot...")
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle('Model Validation Results', fontsize=14)

    # Time domain comparison
    ax = axes[0]
    ax.plot(t_plot, y_meas, 'b-', linewidth=1.5, alpha=0.7, label='Measured')
    ax.plot(t_plot, y_sim, 'r--', linewidth=2, label='Simulated')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Output (Encoder Velocity)')
    ax.set_title(f'Time Domain Comparison (R² = {metrics["r_squared"]:.4f})')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Error plot
    ax = axes[1]
    error = y_meas - y_sim
    ax.plot(t_plot, error, 'g-', linewidth=1, alpha=0.7)
    ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    ax.fill_between(t_plot, error, alpha=0.3)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Error')
    ax.set_title(f'Prediction Error (RMSE = {metrics["rmse"]:.2f})')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    if output_file is None:
        output_file = os.path.splitext(data_file)[0] + '_validation.png'

    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"  Validation plot saved: {output_file}")

    if show:
        plt.show()

    print(f"\n{'='*60}")
    print(f"VALIDATION COMPLETE")
    print(f"{'='*60}\n")

    return validation_data


def main():
    if len(sys.argv) < 3:
        print("Usage: python sysid_validation.py <data.csv> <model.json> [--no-show]")
        print("\nExample:")
        print("  python sysid_validation.py prbs_motor0.csv prbs_motor0_model.json")
        sys.exit(1)

    data_file = sys.argv[1]
    model_file = sys.argv[2]
    show = '--no-show' not in sys.argv

    validate_model(data_file, model_file, show=show)


if __name__ == "__main__":
    main()
