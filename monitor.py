import serial, time, sys
sys.stdout.reconfigure(encoding='utf-8', errors='replace', line_buffering=True)

ser3 = serial.Serial('COM3', 115200, timeout=2)
ser15 = serial.Serial('COM15', 115200, timeout=2)
time.sleep(1)

print("Monitoring COM3 (camera) and COM15 (base)...", flush=True)
try:
    while True:
        cam_out = ser3.read(ser3.in_waiting)
        base_out = ser15.read(ser15.in_waiting)
        if cam_out:
            text = cam_out.decode('utf-8', errors='replace')
            for line in text.split('\n'):
                line = line.strip()
                if line:
                    print(f"[CAM] {line}", flush=True)
        if base_out:
            text = base_out.decode('utf-8', errors='replace')
            for line in text.split('\n'):
                line = line.strip()
                if line:
                    print(f"[BASE] {line}", flush=True)
        time.sleep(0.05)
except KeyboardInterrupt:
    pass
finally:
    ser3.close()
    ser15.close()
