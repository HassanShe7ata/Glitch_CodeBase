"""
ESP32-S3 Vision Viewer (ESP32 computes, PC visualizes)

This viewer only:
1) Reads video frames from ESP32 /stream (or /capture)
2) Polls ESP32 /data for QR detections + pose
3) Draws bold overlays for debugging visibility
"""

import argparse
import time
import threading
import re

import cv2
import numpy as np
import requests


def format_qr_log(qr: dict) -> str:
    text = qr.get("text", "")
    decoded = qr.get("decoded", False)
    estimated = qr.get("estimated", False)
    confidence = qr.get("confidence", 0.0)
    age_ms = qr.get("age_ms", 0)
    tx = qr.get("tx", 0.0)
    ty = qr.get("ty", 0.0)
    tz = qr.get("tz", 0.0)
    roll = qr.get("roll", 0.0)
    pitch = qr.get("pitch", 0.0)
    yaw = qr.get("yaw", 0.0)
    return (
        f"[QR] decoded={decoded} estimated={estimated} conf={confidence:.2f} age_ms={age_ms} text=\"{text}\" "
        f"x={tx:.1f} y={ty:.1f} z={tz:.1f} "
        f"roll={roll:.1f} pitch={pitch:.1f} yaw={yaw:.1f}"
    )


class DataPoller:
    def __init__(self, data_url: str, interval: float = 0.08):
        self._url = data_url
        self._interval = interval
        self._session = requests.Session()
        self._session.headers.update({"Connection": "keep-alive"})
        self._lock = threading.Lock()
        self._data = None
        self._running = False
        self._thread = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

    def get(self):
        with self._lock:
            return self._data

    def _poll_loop(self):
        while self._running:
            try:
                r = self._session.get(self._url, timeout=1.0)
                if r.status_code == 200:
                    d = r.json()
                    with self._lock:
                        self._data = d
            except Exception:
                pass
            time.sleep(self._interval)


class ESP32CamSource:
    def __init__(self, base_url: str, timeout: float = 3.0, prefer_stream: bool = False):
        url = base_url.rstrip("/")
        for sfx in ["/stream", "/capture", "/data", "/status"]:
            if url.endswith(sfx):
                url = url[:-len(sfx)]
                break

        host_base = re.sub(r":\d+$", "", url)
        self.stream_url = host_base + ":81/stream"
        self.capture_url = host_base + "/capture"
        self.timeout = timeout

        self._session = requests.Session()
        self._session.headers.update({"Connection": "keep-alive"})
        self._stream_resp = None
        self._use_stream = prefer_stream
        self._opened = False

    def open(self) -> bool:
        for _ in range(5):
            try:
                if not self._use_stream:
                    r = self._session.get(self.capture_url, timeout=self.timeout)
                    if r.status_code == 200 and len(r.content) > 100:
                        self._opened = True
                        print(f"Connected via capture: {self.capture_url}")
                        return True
            except Exception:
                pass

            try:
                if self._use_stream:
                    r = self._session.get(self.stream_url, stream=True, timeout=self.timeout)
                    if r.status_code == 200:
                        self._stream_resp = r
                        self._opened = True
                        print(f"Connected via MJPEG: {self.stream_url}")
                        return True
                    r.close()
            except Exception:
                pass
            time.sleep(0.3)
        return False

    def read(self):
        if not self._opened:
            return False, None
        if self._use_stream and self._stream_resp:
            return self._read_stream()
        return self._read_capture()

    def _read_stream(self):
        try:
            raw = self._stream_resp.raw
            content_length = None
            line = b""
            for _ in range(4096):
                ch = raw.read(1)
                if not ch:
                    self._reconnect_stream()
                    return False, None
                line += ch
                if line.endswith(b"\r\n"):
                    lower = line.strip().lower()
                    if lower.startswith(b"content-length:"):
                        try:
                            content_length = int(line.split(b":")[1].strip())
                        except ValueError:
                            content_length = None
                    elif lower == b"" and content_length is not None:
                        jpg = raw.read(content_length)
                        if len(jpg) == content_length:
                            arr = np.frombuffer(jpg, dtype=np.uint8)
                            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                            if frame is not None:
                                return True, frame
                        return False, None
                    line = b""
        except Exception:
            self._reconnect_stream()
        return False, None

    def _read_capture(self):
        try:
            r = self._session.get(self.capture_url, timeout=self.timeout)
            if r.status_code == 200 and len(r.content) > 100:
                arr = np.frombuffer(r.content, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame is not None:
                    return True, frame
        except Exception:
            pass
        return False, None

    def _reconnect_stream(self):
        try:
            if self._stream_resp:
                self._stream_resp.close()
        except Exception:
            pass
        self._stream_resp = None
        time.sleep(0.2)
        try:
            r = self._session.get(self.stream_url, stream=True, timeout=self.timeout)
            if r.status_code == 200:
                self._stream_resp = r
        except Exception:
            self._use_stream = False

    def release(self):
        try:
            if self._stream_resp:
                self._stream_resp.close()
            self._session.close()
        except Exception:
            pass


def draw_bold_text(frame, text, org, color, scale=0.8, thick=2):
    # Black outline for readability on bright background.
    cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thick + 3, cv2.LINE_AA)
    cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


def draw_center_lock(frame, cx: int, cy: int, size: int = 64):
    half = size // 2
    x1, y1 = cx - half, cy - half
    x2, y2 = cx + half, cy + half

    # Outer frame that tracks QR center.
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 0), 4)
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 215, 255), 2)

    b = max(10, size // 5)
    c = (0, 255, 255)
    t = 2
    cv2.line(frame, (x1, y1), (x1 + b, y1), c, t)
    cv2.line(frame, (x1, y1), (x1, y1 + b), c, t)
    cv2.line(frame, (x2, y1), (x2 - b, y1), c, t)
    cv2.line(frame, (x2, y1), (x2, y1 + b), c, t)
    cv2.line(frame, (x1, y2), (x1 + b, y2), c, t)
    cv2.line(frame, (x1, y2), (x1, y2 - b), c, t)
    cv2.line(frame, (x2, y2), (x2 - b, y2), c, t)
    cv2.line(frame, (x2, y2), (x2, y2 - b), c, t)

    cv2.drawMarker(frame, (cx, cy), (0, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=16, thickness=2)


def draw_overlay(frame, data):
    if not data or "qr_codes" not in data:
        return 0

    qr_codes = data["qr_codes"]
    hud_qr = None

    for qr in qr_codes:
        corners = np.array(qr.get("corners", []), dtype=np.int32)
        if corners.shape != (4, 2):
            continue

        cv2.polylines(frame, [corners], True, (0, 255, 0), 4)
        for pt in corners:
            cv2.circle(frame, tuple(pt), 7, (0, 220, 255), -1)
            cv2.circle(frame, tuple(pt), 9, (0, 0, 0), 2)

        cx = int(np.mean(corners[:, 0]))
        cy = int(np.mean(corners[:, 1]))
        draw_center_lock(frame, cx, cy, size=64)

        label = qr.get("text", "")[:48]
        draw_bold_text(frame, label, (cx - 120, cy - 22), (50, 255, 50), scale=0.75, thick=2)

        mode = "EST" if qr.get("estimated", False) else "OBS"
        conf = qr.get("confidence", 0.0)
        age_ms = qr.get("age_ms", 0)
        draw_bold_text(
            frame,
            f"{mode}  conf:{conf:.2f}  age:{age_ms}ms",
            (cx - 140, cy - 50),
            (255, 180, 70),
            scale=0.65,
            thick=2,
        )

        if hud_qr is None and qr.get("pose_valid", False):
            hud_qr = qr

    if hud_qr is None and qr_codes:
        hud_qr = qr_codes[0]

    if hud_qr is not None:
        h, w = frame.shape[:2]
        panel_w = 360
        panel_h = 92
        pad = 14
        x1 = max(0, w - panel_w - pad)
        y1 = max(0, h - panel_h - pad)
        x2 = min(w - 1, w - pad)
        y2 = min(h - 1, h - pad)

        glass = frame.copy()
        cv2.rectangle(glass, (x1, y1), (x2, y2), (20, 20, 20), -1)
        cv2.addWeighted(glass, 0.55, frame, 0.45, 0, frame)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 215, 255), 2)

        tx = hud_qr.get("tx", 0.0)
        ty = hud_qr.get("ty", 0.0)
        tz = hud_qr.get("tz", 0.0)
        roll = hud_qr.get("roll", 0.0)
        pitch = hud_qr.get("pitch", 0.0)
        yaw = hud_qr.get("yaw", 0.0)

        draw_bold_text(frame, f"X:{tx:.1f}  Y:{ty:.1f}  Z:{tz:.1f} mm", (x1 + 12, y1 + 36), (0, 220, 255), scale=0.8, thick=2)
        draw_bold_text(frame, f"R:{roll:.1f}  P:{pitch:.1f}  Y:{yaw:.1f} deg", (x1 + 12, y1 + 72), (255, 220, 0), scale=0.8, thick=2)

    return len(qr_codes)


def main():
    parser = argparse.ArgumentParser(description="ESP32 vision viewer")
    parser.add_argument("--url", type=str, default="http://192.168.1.2",
                        help="ESP32 base URL")
    parser.add_argument("--no-display", action="store_true",
                        help="Console only")
    parser.add_argument("--stream", action="store_true",
                        help="Use MJPEG stream mode (higher viewer FPS, more ESP load)")
    parser.add_argument("--log-interval", type=float, default=0.5,
                        help="Seconds between terminal log prints (default: 0.5)")
    parser.add_argument("--data-interval", type=float, default=0.08,
                        help="Seconds between /data polls (default: 0.08)")
    parser.add_argument("--frame-interval", type=float, default=0.20,
                        help="Minimum seconds between frame fetches in capture mode (default: 0.20)")
    args = parser.parse_args()

    base_url = args.url.rstrip("/")
    for suffix in ["/capture", "/stream", "/data", "/status"]:
        if base_url.endswith(suffix):
            base_url = base_url[:-len(suffix)]
            break

    data_url = base_url + "/data"

    print("ESP32-S3 Vision Viewer")
    print(f"  Camera: {base_url}")
    print(f"  Data:   {data_url}")
    print("  Compute: ESP32 only")

    poller = DataPoller(data_url, interval=args.data_interval)
    poller.start()

    cam = None
    if not args.no_display:
        cam = ESP32CamSource(base_url, prefer_stream=args.stream)
        if not cam.open():
            print("ERROR: cannot connect camera")
            poller.stop()
            return

    frame_count = 0
    fps_time = time.time()
    fps = 0.0
    last_data = None
    last_log_t = 0.0
    last_noqr_t = 0.0
    next_frame_t = 0.0
    frame = None

    try:
        while True:
            now = time.time()

            if not args.no_display:
                if now < next_frame_t:
                    time.sleep(0.005)
                    continue

                ret, frame = cam.read()
                if not ret or frame is None:
                    time.sleep(0.03)
                    continue

                if not args.stream and args.frame_interval > 0:
                    next_frame_t = now + args.frame_interval

            d = poller.get()
            if d is not None:
                last_data = d

            if last_data and last_data.get("qr_codes") and (now - last_log_t) >= args.log_interval:
                for qr in last_data["qr_codes"]:
                    print(format_qr_log(qr))
                last_log_t = now
            elif (not last_data or not last_data.get("qr_codes")) and (now - last_noqr_t) >= 2.0:
                print("[QR] no detection")
                last_noqr_t = now

            n_qr = 0
            if not args.no_display:
                n_qr = draw_overlay(frame, last_data)

                frame_count += 1
                elapsed = time.time() - fps_time
                if elapsed >= 1.0:
                    fps = frame_count / elapsed
                    frame_count = 0
                    fps_time = time.time()

                h, w = frame.shape[:2]
                draw_bold_text(frame, f"PC FPS: {fps:.1f}", (w - 210, 30), (0, 255, 0), scale=0.8, thick=2)
                draw_bold_text(frame, f"QR: {n_qr}", (10, h - 16), (255, 255, 255), scale=0.8, thick=2)

                if last_data:
                    draw_bold_text(
                        frame,
                        f"ESP: {last_data.get('qr_fps', 0):.1f}fps  {last_data.get('processing_ms', 0)}ms",
                        (10, 30),
                        (255, 210, 40),
                        scale=0.75,
                        thick=2,
                    )

                cv2.imshow("ESP32-S3 Vision (On-device Compute)", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord("s"):
                    fname = f"snapshot_{int(time.time())}.jpg"
                    cv2.imwrite(fname, frame)
                    print(f"Saved: {fname}")
            else:
                time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nInterrupted by user. Exiting viewer...")
    finally:
        poller.stop()
        if cam:
            cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
