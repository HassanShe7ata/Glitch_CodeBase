"""Serial monitor — reads from a COM port at 115200 and logs to file."""
import sys, serial, datetime

port = sys.argv[1] if len(sys.argv) > 1 else "COM15"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
tag = sys.argv[3] if len(sys.argv) > 3 else "log"

ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
logfile = f"D:\\Glitch_Codes\\Glitch_CodeBase\\{tag}_{ts}.log"

print(f"Opening {port} @ {baud} — logging to {logfile}")
print("Press Ctrl+C to stop.\n")

ser = serial.Serial(port, baud, timeout=1)
with open(logfile, "w", encoding="utf-8") as f:
    try:
        while True:
            raw = ser.readline()
            if raw:
                line = raw.decode("utf-8", errors="replace").rstrip()
                stamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                tagged = f"[{stamp}] {line}"
                print(tagged)
                f.write(tagged + "\n")
                f.flush()
    except KeyboardInterrupt:
        print(f"\nStopped. Log saved: {logfile}")
    finally:
        ser.close()
