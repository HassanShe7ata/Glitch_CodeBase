"""
speed_response_plot.py - Speed Verification Curve
Glitch Robot Project

Generates setpoint vs actual speed plot from captured data.
"""

import csv
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict

# Load data
data_file = r'D:\Glitch_Codes\Glitch_CodeBase\SystemIdentification\speed_response_clean.csv'

timestamps = []
setpoint_pct = []
setpoint_kmh = []
actual_kmh = []

with open(data_file, 'r') as f:
    reader = csv.reader(f)
    next(reader)  # skip header
    for row in reader:
        timestamps.append(int(row[0]))
        setpoint_pct.append(int(row[1]))
        setpoint_kmh.append(float(row[2]))
        actual_kmh.append(float(row[3]))

timestamps = np.array(timestamps)
setpoint_kmh = np.array(setpoint_kmh)
actual_kmh = np.array(actual_kmh)

# EMA filter for smoothing
alpha = 0.1
actual_filtered = np.zeros_like(actual_kmh)
actual_filtered[0] = actual_kmh[0]
for i in range(1, len(actual_kmh)):
    actual_filtered[i] = alpha * actual_kmh[i] + (1 - alpha) * actual_filtered[i-1]

# Create figure with 2x2 layout
fig, axes = plt.subplots(2, 2, figsize=(14, 10))
fig.suptitle('Speed Response Verification - Glitch Robot Motor', fontsize=14, fontweight='bold')

# Plot 1: Time domain - Setpoint vs Actual
ax = axes[0, 0]
ax.plot(timestamps/1000, setpoint_kmh, 'b-', linewidth=2, label='Setpoint', alpha=0.8)
ax.plot(timestamps/1000, actual_kmh, 'r-', linewidth=0.8, alpha=0.4, label='Actual (raw)')
ax.plot(timestamps/1000, actual_filtered, 'r-', linewidth=2, label='Actual (filtered)')
ax.set_xlabel('Time [s]')
ax.set_ylabel('Speed [km/hr]')
ax.set_title('Time Domain Response')
ax.legend()
ax.grid(True, alpha=0.3)

# Plot 2: Steady-state setpoint vs actual
ax = axes[0, 1]
levels = defaultdict(list)
for i in range(len(timestamps)):
    levels[setpoint_pct[i]].append(actual_kmh[i])

setpoints_steady = []
actuals_steady = []
actuals_std = []
for pct in sorted(levels.keys()):
    samples = levels[pct]
    steady_start = int(len(samples) * 0.3)
    steady = samples[steady_start:]
    if steady and pct > 0:
        setpoint_val = setpoint_kmh[setpoint_pct.index(pct)]
        setpoints_steady.append(setpoint_val)
        actuals_steady.append(np.mean(steady))
        actuals_std.append(np.std(steady))

ax.errorbar(setpoints_steady, actuals_steady, yerr=actuals_std, fmt='ro', markersize=8,
            capsize=5, capthick=2, label='Measured')
# Perfect tracking line
max_val = max(max(setpoints_steady), max(actuals_steady)) * 1.1
ax.plot([0, max_val], [0, max_val], 'b--', linewidth=2, label='Perfect tracking')
ax.set_xlabel('Setpoint Speed [km/hr]')
ax.set_ylabel('Actual Speed [km/hr]')
ax.set_title('Steady-State Speed Tracking')
ax.legend()
ax.grid(True, alpha=0.3)
ax.set_xlim([0, max_val])
ax.set_ylim([0, max_val])
ax.set_aspect('equal')

# Plot 3: Tracking error vs setpoint
ax = axes[1, 0]
errors = [(a - s) / s * 100 if s > 0 else 0 for s, a in zip(setpoints_steady, actuals_steady)]
ax.bar(range(len(errors)), errors, color=['green' if abs(e) < 20 else 'red' for e in errors])
ax.set_xticks(range(len(errors)))
ax.set_xticklabels(['%.1f' % s for s in setpoints_steady], rotation=45)
ax.set_xlabel('Setpoint Speed [km/hr]')
ax.set_ylabel('Tracking Error [%]')
ax.set_title('Speed Tracking Error')
ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
ax.grid(True, alpha=0.3, axis='y')

# Plot 4: Step response for each speed level
ax = axes[1, 1]
colors = plt.cm.viridis(np.linspace(0, 1, len(levels)))
for i, pct in enumerate(sorted(levels.keys())):
    if pct == 0:
        continue
    samples = levels[pct]
    t_local = np.arange(len(samples)) * 10  # 10ms per sample
    act_local = np.array(samples)
    # Filter
    act_filt = np.zeros_like(act_local)
    act_filt[0] = act_local[0]
    for j in range(1, len(act_local)):
        act_filt[j] = alpha * act_local[j] + (1 - alpha) * act_filt[j-1]
    setpoint_val = setpoint_kmh[setpoint_pct.index(pct)]
    ax.plot(t_local/1000, act_filt, color=colors[i], linewidth=1.5, label='%d%% (%.1f km/h)' % (pct, setpoint_val))
    ax.axhline(y=setpoint_val, color=colors[i], linestyle='--', alpha=0.3)

ax.set_xlabel('Time [s]')
ax.set_ylabel('Actual Speed [km/hr]')
ax.set_title('Step Response per Speed Level')
ax.legend(fontsize=8, loc='upper left')
ax.grid(True, alpha=0.3)

plt.tight_layout()
output_plot = r'D:\Glitch_Codes\Glitch_CodeBase\SystemIdentification\speed_verification_curve.png'
plt.savefig(output_plot, dpi=150, bbox_inches='tight')
print('Saved: %s' % output_plot)
plt.show()
