import serial
import time
import sys

PORT = 'COM15'
BAUD = 115200
OUTFILE = r'D:\Glitch_Codes\Glitch_CodeBase\SystemIdentification\speed_verify_data.csv'

ser = serial.Serial(PORT, BAUD, timeout=2)
time.sleep(3)

print("Capturing speed verification data...")
lines = []
header = None

start = time.time()
while time.time() - start < 60:
    raw = ser.readline()
    if not raw:
        continue
    line = raw.decode('utf-8', errors='replace').strip()
    if not line:
        continue
    if line.startswith('#'):
        print(line)
        continue
    if line.startswith('Time_ms'):
        header = line
        print('HEADER:', header)
        continue
    if 'DONE' in line:
        print(line)
        break
    lines.append(line)
    if len(lines) % 50 == 0:
        print(f'  samples: {len(lines)}')

ser.close()

with open(OUTFILE, 'w') as f:
    if header:
        f.write(header + '\n')
    for l in lines:
        f.write(l + '\n')

print(f'Saved {len(lines)} samples to {OUTFILE}')
