"""Serve the camera as an MJPEG stream with crack detections and publish them over MQTT.

Video goes over HTTP (the dashboard browser loads it directly) and only the
detection summary goes over MQTT, so video traffic never competes with the
robot command topics on the broker.

Endpoints:
    /stream.mjpg   live annotated video (multipart MJPEG)
    /snapshot.jpg  latest annotated frame
    /health        JSON with frame rates and the last detection
"""
import argparse
import json
import socket
import sys
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import paho.mqtt.client as mqtt
from ultralytics import YOLO

from crack_poc import build_message
from webcam_poc import draw_status

BOUNDARY = b"frame"


class Latest:
    """Holds the newest value and lets readers block until a newer one arrives."""

    def __init__(self):
        self._condition = threading.Condition()
        self._value = None
        self._seq = 0

    def put(self, value):
        with self._condition:
            self._value = value
            self._seq += 1
            self._condition.notify_all()

    def get(self):
        with self._condition:
            return self._seq, self._value

    def wait_newer(self, seq, timeout):
        """Return (seq, value); the returned seq equals the given one on timeout."""
        with self._condition:
            self._condition.wait_for(lambda: self._seq != seq, timeout)
            return self._seq, self._value


class RateMeter:
    """Exponential moving average of events per second."""

    def __init__(self, smoothing=0.9):
        self.smoothing = smoothing
        self.rate = 0.0
        self._last = None

    def tick(self):
        now = time.perf_counter()
        if self._last is not None:
            instant = 1.0 / max(now - self._last, 1e-6)
            self.rate = instant if self.rate == 0 else self.smoothing * self.rate + (1 - self.smoothing) * instant
        self._last = now


class VisionPublisher:
    """Publishes detection summaries, throttled, plus a retained online/offline status."""

    def __init__(self, args):
        self.topic = args.topic
        self.status_topic = f"{args.topic}/status"
        self.interval = args.publish_interval
        self._last_sent = 0.0
        self._last_detected = None
        client_id = f"vision-{socket.gethostname()}-{uuid.uuid4().hex[:6]}"
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
        self.client.will_set(self.status_topic, "offline", qos=1, retain=True)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.connect_async(args.broker, args.mqtt_port, keepalive=30)
        self.client.loop_start()
        self._broker = f"{args.broker}:{args.mqtt_port}"

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            print(f"MQTT connection refused: {reason_code}")
            return
        print(f"MQTT connected to {self._broker}, publishing on {self.topic}")
        client.publish(self.status_topic, "online", qos=1, retain=True)

    def _on_disconnect(self, client, userdata, flags, reason_code, properties):
        print(f"MQTT disconnected: {reason_code}")

    def maybe_publish(self, message):
        """Send on every detected/not-detected change, otherwise at most once per interval."""
        now = time.monotonic()
        changed = message["detected"] != self._last_detected
        if not changed and now - self._last_sent < self.interval:
            return
        self._last_sent = now
        self._last_detected = message["detected"]
        self.client.publish(self.topic, json.dumps(message), qos=0)
        if changed:
            print(json.dumps(message))

    def close(self):
        if self.client.is_connected():
            self.client.publish(self.status_topic, "offline", qos=1, retain=True).wait_for_publish(2)
        self.client.disconnect()
        self.client.loop_stop()


class OfflinePublisher:
    """Stand-in used with --no-mqtt: only logs detection changes."""

    def __init__(self):
        self._last_detected = None

    def maybe_publish(self, message):
        if message["detected"] != self._last_detected:
            self._last_detected = message["detected"]
            print(json.dumps(message))

    def close(self):
        pass


def open_camera(source):
    """Open a camera index ("0") or any OpenCV source string (file, RTSP/HTTP URL)."""
    if source.isdigit():
        backend = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY
        capture = cv2.VideoCapture(int(source), backend)
    else:
        capture = cv2.VideoCapture(source)
    if not capture.isOpened():
        raise SystemExit(f"Could not open camera source {source!r}")
    return capture


def capture_loop(capture, frames, meter, stop):
    failures = 0
    while not stop.is_set():
        ok, frame = capture.read()
        if not ok:
            failures += 1
            if failures > 50:
                print("Camera stopped delivering frames")
                stop.set()
            stop.wait(0.05)
            continue
        failures = 0
        meter.tick()
        frames.put(frame)


def extract_shapes(result):
    polygons = []
    if result.masks is not None:
        polygons = [p.astype(np.int32) for p in result.masks.xy if len(p)]
    boxes = result.boxes.xyxy.int().tolist() if result.boxes is not None else []
    return polygons, boxes


def draw_detections(frame, detection, camera_fps, inference_fps):
    """Draw the latest detection on a (possibly newer) camera frame."""
    for polygon in detection["polygons"]:
        cv2.polylines(frame, [polygon], True, (0, 0, 255), 2)
    for x1, y1, x2, y2 in detection["boxes"]:
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 165, 255), 1)
    draw_status(frame, detection["message"])
    rates = f"cam {camera_fps:.0f} fps | ai {inference_fps:.1f} fps"
    (width, _), _ = cv2.getTextSize(rates, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    cv2.putText(frame, rates, (frame.shape[1] - width - 10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1)


def compose_loop(frames, detections, jpegs, meters, args, stop):
    """Encode every camera frame once, so stream clients only copy bytes."""
    seq = 0
    while not stop.is_set():
        new_seq, frame = frames.wait_newer(seq, timeout=1.0)
        if new_seq == seq:
            continue
        seq = new_seq
        canvas = frame.copy()
        _, detection = detections.get()
        if detection is not None:
            draw_detections(canvas, detection, meters["camera"].rate, meters["inference"].rate)
        if args.stream_width and canvas.shape[1] > args.stream_width:
            height = round(canvas.shape[0] * args.stream_width / canvas.shape[1])
            canvas = cv2.resize(canvas, (args.stream_width, height), interpolation=cv2.INTER_AREA)
        ok, jpeg = cv2.imencode(".jpg", canvas, [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality])
        if ok:
            jpegs.put(jpeg.tobytes())


def inference_loop(model, frames, detections, publisher, meters, args, stop):
    """Run the model on the newest frame only; frames that arrive meanwhile are skipped."""
    min_interval = 1.0 / args.max_infer_fps if args.max_infer_fps > 0 else 0.0
    seq = 0
    while not stop.is_set():
        new_seq, frame = frames.wait_newer(seq, timeout=1.0)
        if new_seq == seq:
            continue
        seq = new_seq
        started = time.perf_counter()
        result = model.predict(frame, imgsz=args.imgsz, conf=args.conf, device=args.device, verbose=False)[0]
        elapsed_ms = (time.perf_counter() - started) * 1000
        meters["inference"].tick()

        message = {
            **build_message(result, elapsed_ms),
            "ts": int(time.time() * 1000),
            "cameraFps": round(meters["camera"].rate, 1),
            "inferenceFps": round(meters["inference"].rate, 1),
            "frameWidth": frame.shape[1],
            "frameHeight": frame.shape[0],
            "host": socket.gethostname(),
        }
        polygons, boxes = extract_shapes(result)
        detections.put({"message": message, "polygons": polygons, "boxes": boxes})
        publisher.maybe_publish(message)

        if min_interval:
            stop.wait(max(0.0, min_interval - (time.perf_counter() - started)))


def make_handler(jpegs, health, stop):
    class StreamHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path == "/stream.mjpg":
                self._stream()
            elif path == "/snapshot.jpg":
                self._snapshot()
            elif path == "/health":
                self._send(200, "application/json", json.dumps(health()).encode())
            else:
                self.send_error(404)

        def _send(self, status, content_type, body):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)

        def _snapshot(self):
            _, jpeg = jpegs.get()
            if jpeg is None:
                self.send_error(503, "No frame yet")
                return
            self._send(200, "image/jpeg", jpeg)

        def _stream(self):
            self.send_response(200)
            self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY.decode()}")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            seq = 0
            try:
                while not stop.is_set():
                    new_seq, jpeg = jpegs.wait_newer(seq, timeout=1.0)
                    if new_seq == seq:
                        continue
                    seq = new_seq
                    header = b"--%s\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % (BOUNDARY, len(jpeg))
                    self.wfile.write(header + jpeg + b"\r\n")
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass  # The viewer closed the page.

        def log_message(self, format, *args):
            pass  # Keep the console for detection output.

    return StreamHandler


def lan_address():
    """Best-effort LAN IP for the startup message (UDP connect sends no packets)."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        try:
            probe.connect(("8.8.8.8", 80))
            return probe.getsockname()[0]
        except OSError:
            return "localhost"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--camera", default="0", help="Camera index or OpenCV source URL/path")
    parser.add_argument("--weights", default="weights/yolov8n-crack.pt")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--max-infer-fps", type=float, default=0.0, help="Cap inference rate to free CPU (0 = no cap)")
    parser.add_argument("--http-host", default="0.0.0.0")
    parser.add_argument("--http-port", type=int, default=8000)
    parser.add_argument("--stream-width", type=int, default=640, help="Downscale streamed frames wider than this")
    parser.add_argument("--jpeg-quality", type=int, default=70)
    parser.add_argument("--broker", default="broker.hivemq.com")
    parser.add_argument("--mqtt-port", type=int, default=1883)
    parser.add_argument("--topic", default="robot/vision")
    parser.add_argument("--publish-interval", type=float, default=0.5, help="Seconds between unchanged detection messages")
    parser.add_argument("--no-mqtt", action="store_true", help="Only stream video; log detections locally")
    return parser.parse_args()


def main():
    args = parse_args()
    stop = threading.Event()
    frames, detections, jpegs = Latest(), Latest(), Latest()
    meters = {"camera": RateMeter(), "inference": RateMeter()}

    capture = open_camera(args.camera)
    model = YOLO(args.weights)
    print("Warming up model...")
    # The first predict call is many times slower; keep it out of the published metrics.
    model.predict(np.zeros((480, 640, 3), np.uint8), imgsz=args.imgsz, device=args.device, verbose=False)
    publisher = OfflinePublisher() if args.no_mqtt else VisionPublisher(args)

    def health():
        _, detection = detections.get()
        return {
            "cameraFps": round(meters["camera"].rate, 1),
            "inferenceFps": round(meters["inference"].rate, 1),
            "lastDetection": detection["message"] if detection else None,
        }

    server = ThreadingHTTPServer((args.http_host, args.http_port), make_handler(jpegs, health, stop))
    server.daemon_threads = True
    workers = [
        threading.Thread(target=capture_loop, args=(capture, frames, meters["camera"], stop), daemon=True),
        threading.Thread(target=compose_loop, args=(frames, detections, jpegs, meters, args, stop), daemon=True),
        threading.Thread(target=server.serve_forever, daemon=True),
    ]
    for worker in workers:
        worker.start()
    print(f"Stream:   http://{lan_address()}:{args.http_port}/stream.mjpg")
    print(f"Health:   http://localhost:{args.http_port}/health")
    print("Press Ctrl+C to stop")

    try:
        inference_loop(model, frames, detections, publisher, meters, args, stop)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.shutdown()
        server.server_close()
        workers[0].join(timeout=2)  # Let the capture thread leave read() before releasing.
        capture.release()
        publisher.close()


if __name__ == "__main__":
    main()
