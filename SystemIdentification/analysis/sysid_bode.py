"""
sysid_bode.py - Bode Plot Generation
Glitch Robot Project

Generates publication-quality Bode plots from system identification data.

Usage:
    python sysid_bode.py <model.json> [--output plot.png]
"""

import numpy as np
from scipy import signal
import matplotlib.pyplot as plt
import json
import sys
import os


def load_model(filename):
    """Load system identification model from JSON."""
    with open(filename, 'r') as f:
        return json.load(f)


def generate_bode_plot(model, output_file=None, show=True):
    """Generate publication-quality Bode plot."""
    # Extract transfer function
    if 'transfer_function' not in model:
        print("Error: No transfer function found in model.")
        return None

    tf = model['transfer_function']
    num = np.array(tf['numerator'])
    den = np.array(tf['denominator'])

    # Extract measured data
    freq_resp = model['frequency_response']
    f_meas = np.array(freq_resp['frequency_hz'])
    mag_meas = np.array(freq_resp['magnitude'])
    phase_meas = np.array(freq_resp['phase_deg'])

    # Generate fitted response
    w = 2 * np.pi * f_meas
    _, H_fit = signal.freqs(num, den, worN=w)
    mag_fit = np.abs(H_fit)
    phase_fit = np.angle(H_fit, deg=True)

    # Create figure
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # Magnitude plot
    ax1.semilogx(f_meas, 20 * np.log10(mag_meas + 1e-20), 'b-', linewidth=1.5,
                 alpha=0.7, label='Measured')
    ax1.semilogx(f_meas, 20 * np.log10(mag_fit + 1e-20), 'r--', linewidth=2,
                 label=f'Fitted (TF order: {tf["order_n"]}/{tf["order_d"]})')
    ax1.set_ylabel('Magnitude [dB]', fontsize=12)
    ax1.set_title('Motor Frequency Response (Bode Plot)', fontsize=14)
    ax1.legend(fontsize=10)
    ax1.grid(True, which='both', linestyle='--', alpha=0.7)
    ax1.set_xlim([f_meas[0], f_meas[-1]])

    # Phase plot
    ax2.semilogx(f_meas, phase_meas, 'b-', linewidth=1.5, alpha=0.7, label='Measured')
    ax2.semilogx(f_meas, phase_fit, 'r--', linewidth=2, label='Fitted')
    ax2.set_xlabel('Frequency [Hz]', fontsize=12)
    ax2.set_ylabel('Phase [deg]', fontsize=12)
    ax2.legend(fontsize=10)
    ax2.grid(True, which='both', linestyle='--', alpha=0.7)
    ax2.set_xlim([f_meas[0], f_meas[-1]])

    # Add grid for phase at multiples of 90
    ax2.set_yticks([-180, -90, 0, 90, 180])

    plt.tight_layout()

    if output_file is None:
        output_file = os.path.splitext(
            sys.argv[1] if len(sys.argv) > 1 else 'bode_plot'
        )[0] + '_bode.png'

    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Bode plot saved: {output_file}")

    if show:
        plt.show()

    return fig


def generate_pole_zero_plot(model, output_file=None, show=True):
    """Generate pole-zero plot."""
    if 'transfer_function' not in model:
        print("Error: No transfer function found in model.")
        return None

    tf = model['transfer_function']
    num = np.array(tf['numerator'])
    den = np.array(tf['denominator'])

    zeros = np.roots(num)
    poles = np.roots(den)

    fig, ax = plt.subplots(1, 1, figsize=(8, 8))

    # Plot unit circle
    theta = np.linspace(0, 2*np.pi, 100)
    ax.plot(np.cos(theta), np.sin(theta), 'k--', alpha=0.3, label='Unit Circle')

    # Plot imaginary axis
    ax.axhline(y=0, color='k', linewidth=0.5)
    ax.axvline(x=0, color='k', linewidth=0.5)

    # Plot zeros and poles
    if len(zeros) > 0:
        ax.plot(np.real(zeros), np.imag(zeros), 'bo', markersize=10, label='Zeros', markerfacecolor='none', markeredgewidth=2)
    if len(poles) > 0:
        ax.plot(np.real(poles), np.imag(poles), 'rx', markersize=12, label='Poles', markeredgewidth=2)

    ax.set_xlabel('Real', fontsize=12)
    ax.set_ylabel('Imaginary', fontsize=12)
    ax.set_title('Pole-Zero Map', fontsize=14)
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    # Auto-scale with margin
    all_points = np.concatenate([zeros, poles])
    if len(all_points) > 0:
        max_abs = max(abs(all_points.max()), abs(all_points.min()), 1.0)
        ax.set_xlim([-max_abs * 1.2, max_abs * 1.2])
        ax.set_ylim([-max_abs * 1.2, max_abs * 1.2])

    plt.tight_layout()

    if output_file is None:
        output_file = os.path.splitext(
            sys.argv[1] if len(sys.argv) > 1 else 'pz_plot'
        )[0] + '_poles_zeros.png'

    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Pole-zero plot saved: {output_file}")

    if show:
        plt.show()

    return fig


def main():
    if len(sys.argv) < 2:
        print("Usage: python sysid_bode.py <model.json> [--no-show]")
        print("\nExample:")
        print("  python sysid_bode.py prbs_motor0_model.json")
        sys.exit(1)

    model = load_model(sys.argv[1])
    show = '--no-show' not in sys.argv

    # Generate Bode plot
    generate_bode_plot(model, show=show)

    # Generate pole-zero plot
    generate_pole_zero_plot(model, show=show)


if __name__ == "__main__":
    main()
