import argparse
import json
import statistics
import time
import requests


def main():
    p = argparse.ArgumentParser(description="Evaluate ESP32 QR session quality")
    p.add_argument("--url", type=str, default="http://192.168.1.2")
    p.add_argument("--duration", type=float, default=20.0)
    p.add_argument("--interval", type=float, default=0.2)
    p.add_argument("--json", action="store_true", help="Print metrics as one JSON object")
    args = p.parse_args()

    base = args.url.rstrip("/")
    for suffix in ["/capture", "/stream", "/data", "/status"]:
        if base.endswith(suffix):
            base = base[:-len(suffix)]
            break

    data_url = base + "/data"
    status_url = base + "/status"

    ses = requests.Session()
    t_end = time.time() + args.duration

    samples = 0
    qr_present = 0
    decoded_true = 0
    conf_vals = []
    tz_vals = []
    proc_ms_vals = []
    status_ok = 0

    while time.time() < t_end:
        samples += 1
        try:
            d = ses.get(data_url, timeout=2.0).json()
            qrs = d.get("qr_codes", [])
            if qrs:
                qr_present += 1
                q = qrs[0]
                if q.get("decoded", False):
                    decoded_true += 1
                conf_vals.append(float(q.get("confidence", 0.0)))
                tz_vals.append(float(q.get("tz", 0.0)))
            proc_ms_vals.append(float(d.get("processing_ms", 0.0)))
        except Exception:
            pass

        try:
            s = ses.get(status_url, timeout=2.0).json()
            if s.get("camera") == "OK":
                status_ok += 1
        except Exception:
            pass

        time.sleep(args.interval)

    det_ratio = (qr_present / samples) if samples else 0.0
    dec_ratio = (decoded_true / qr_present) if qr_present else 0.0

    metrics = {
        "samples": samples,
        "detection_ratio": det_ratio,
        "decode_ratio_when_detected": dec_ratio,
        "camera_status_ok_ratio": (status_ok / samples if samples else 0.0),
        "confidence_mean": (statistics.fmean(conf_vals) if conf_vals else None),
        "tz_mean_mm": (statistics.fmean(tz_vals) if tz_vals else None),
        "tz_std_mm": (statistics.pstdev(tz_vals) if tz_vals else None),
        "proc_ms_mean": (statistics.fmean(proc_ms_vals) if proc_ms_vals else None),
        "proc_ms_p95": (sorted(proc_ms_vals)[int(0.95 * (len(proc_ms_vals) - 1))] if proc_ms_vals else None),
    }

    if args.json:
        print(json.dumps(metrics, separators=(",", ":")))
        return

    print("=== SESSION METRICS ===")
    print(f"samples={metrics['samples']}")
    print(f"detection_ratio={metrics['detection_ratio']:.3f}")
    print(f"decode_ratio_when_detected={metrics['decode_ratio_when_detected']:.3f}")
    print(f"camera_status_ok_ratio={metrics['camera_status_ok_ratio']:.3f}")
    if metrics["confidence_mean"] is not None:
        print(f"confidence_mean={metrics['confidence_mean']:.3f}")
    if metrics["tz_mean_mm"] is not None:
        print(f"tz_mean_mm={metrics['tz_mean_mm']:.2f}")
        print(f"tz_std_mm={metrics['tz_std_mm']:.2f}")
    if metrics["proc_ms_mean"] is not None:
        print(f"proc_ms_mean={metrics['proc_ms_mean']:.1f}")
        print(f"proc_ms_p95={metrics['proc_ms_p95']:.1f}")


if __name__ == "__main__":
    main()
