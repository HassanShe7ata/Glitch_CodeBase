"""
sysid_capture.py - Serial Data Capture for System Identification
Glitch Robot Project

Captures PRBS/Step/Sweep test data from ESP32 serial output.
Parses CSV format and saves to file with metadata.

Usage:
    python sysid_capture.py [COM_PORT] [BAUD_RATE] [OUTPUT_FILE]

Example:
    python sysid_capture.py COM15 115200 prbs_motor0.csv
"""

import sys
import serial
import datetime
import time
import re


def parse_csv_line(line):
    """Parse a CSV line and extract numeric values."""
    try:
        parts = line.strip().split(',')
        if len(parts) >= 3:
            timestamp = int(parts[0])
            pwm = int(parts[1])
            encoder = int(parts[2])
            velocity = int(parts[3]) if len(parts) > 3 else 0
            return timestamp, pwm, encoder, velocity
    except (ValueError, IndexError):
        pass
    return None


def extract_metadata(lines):
    """Extract test metadata from header lines."""
    metadata = {}
    for line in lines:
        if line.startswith('#'):
            line = line[2:].strip()
            if ':' in line:
                key, value = line.split(':', 1)
                metadata[key.strip()] = value.strip()
    return metadata


def capture_data(port, baud, output_file, timeout_s=30):
    """Capture serial data and save to CSV file."""
    print(f"Opening {port} @ {baud} baud...")
    print(f"Output file: {output_file}")
    print("Press Ctrl+C to stop.\n")

    ser = serial.Serial(port, baud, timeout=1)
    time.sleep(2)  # Wait for connection

    header_lines = []
    data_lines = []
    metadata = {}
    capturing = False

    start_time = time.time()

    try:
        while True:
            raw = ser.readline()
            if raw:
                line = raw.decode('utf-8', errors='replace').strip()

                if not line:
                    continue

                # Capture header/metadata
                if line.startswith('#'):
                    header_lines.append(line)
                    print(f"  {line}")
                    continue

                # CSV header line
                if line.startswith('Timestamp_ms'):
                    capturing = True
                    print(f"\n  [CAPTURING DATA] {line}")
                    continue

                # Test complete marker
                if 'COMPLETE' in line.upper():
                    print(f"\n  [TEST COMPLETE]")
                    break

                # Parse data line
                if capturing:
                    parsed = parse_csv_line(line)
                    if parsed:
                        data_lines.append(parsed)
                        # Print progress every 100 samples
                        if len(data_lines) % 100 == 0:
                            elapsed = parsed[0] / 1000.0
                            print(f"  Samples: {len(data_lines)}, Time: {elapsed:.1f}s", end='\r')

            # Timeout check
            if time.time() - start_time > timeout_s:
                print(f"\n  [TIMEOUT] Captured {len(data_lines)} samples")
                break

    except KeyboardInterrupt:
        print(f"\n\n  [STOPPED] Captured {len(data_lines)} samples")
    finally:
        ser.close()

    # Extract metadata from headers
    metadata = extract_metadata(header_lines)

    # Save to file
    if data_lines:
        with open(output_file, 'w') as f:
            # Write metadata
            f.write("# System Identification Data\n")
            f.write(f"# Capture Date: {datetime.datetime.now().isoformat()}\n")
            for key, value in metadata.items():
                f.write(f"# {key}: {value}\n")
            f.write("#\n")

            # Write CSV header
            f.write("Timestamp_ms,Input_PWM,Encoder_Ticks,Encoder_Velocity\n")

            # Write data
            for timestamp, pwm, encoder, velocity in data_lines:
                f.write(f"{timestamp},{pwm},{encoder},{velocity}\n")

        print(f"\n  Saved {len(data_lines)} samples to {output_file}")

        # Print summary
        if data_lines:
            durations = [d[0] for d in data_lines]
            pwms = [d[1] for d in data_lines]
            print(f"  Duration: {max(durations)/1000:.2f}s")
            print(f"  PWM range: {min(pwms)} to {max(pwms)}")
            print(f"  Sample rate: {len(data_lines) / (max(durations)/1000):.1f} Hz")
    else:
        print("  No data captured.")

    return data_lines, metadata


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM15"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    output_file = sys.argv[3] if len(sys.argv) > 3 else f"prbs_data_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

    capture_data(port, baud, output_file)


if __name__ == "__main__":
    main()
