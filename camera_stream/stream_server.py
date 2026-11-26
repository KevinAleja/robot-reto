"""Re-serve the robot camera (ESP32-CAM MJPEG) to any number of browsers, raw and filtered.

The ESP32-CAM usually accepts a single stream client, so this laptop process is
its only consumer. It decodes every frame once, re-encodes it and serves it to
every viewer (the phone HUD included). A second, independent stage applies the
active crack/edge filter (see filters.py) only while someone watches it.

Sources:
    http://<esp32-ip>:81/stream   ESP32-CAM CameraWebServer MJPEG (or any OpenCV URL)
    0, 1, ...                     local webcam index, for testing without the robot
    test                          synthetic moving test pattern, no hardware needed

Endpoints:
    /raw.mjpg                         live video, untouched by the filter stage
    /filtered.mjpg                    live video with the active filter
    /snapshot.jpg?view=raw|filtered   latest frame (default raw)
    /health                           JSON: source, connected, fps, filter, filterMs

MQTT:
    equipo1/robot/vision/filter (subscribed, retained)  "canny" or {"filter": "canny", "params": {...}}
    equipo1/robot/vision/status (retained)              {"state": "online", "fps", "connected", "filter", "filterMs", ...}
                                                LWT {"state": "offline"}
"""
import argparse
import json
import logging
import os
import socket
import sys
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

import cv2
import numpy as np
import paho.mqtt.client as mqtt

from filters import MODES, apply_filter, merge_params, parse_filter_command, resolve_mode

BOUNDARY = b"frame"
STALE_FRAME_S = 2.0


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
        self.last = None

    def tick(self):
        now = time.monotonic()
        if self.last is not None:
            instant = 1.0 / max(now - self.last, 1e-6)
            self.rate = instant if self.rate == 0 else self.smoothing * self.rate + (1 - self.smoothing) * instant
        self.last = now

    def reset(self):
        self.rate = 0.0
        self.last = None


class FilterState:
    """Thread-safe active filter, its params, cost statistics and viewer count."""

    def __init__(self, mode="none"):
        self._lock = threading.Lock()
        self._mode = resolve_mode(mode)
        self._params = {}
        self._merged = merge_params()
        self._ms = None
        self._ms_at = 0.0
        self.viewers = 0
        self.listeners = []

    def get(self):
        """Return (mode, user params, merged params)."""
        with self._lock:
            return self._mode, dict(self._params), self._merged

    def set(self, mode, params=None):
        merged = merge_params(params)  # Raises before switching if params are unusable.
        mode = resolve_mode(mode)
        with self._lock:
            self._mode, self._params, self._merged = mode, dict(params or {}), merged
            self._ms = None
        print(f"Filter set to {mode} {json.dumps(params or {})}")
        for listener in self.listeners:
            listener()

    def record(self, elapsed_ms):
        with self._lock:
            self._ms = elapsed_ms if self._ms is None else 0.9 * self._ms + 0.1 * elapsed_ms
            self._ms_at = time.monotonic()

    def filter_ms(self):
        """Average filter cost per frame, or None when the filter has not run recently."""
        with self._lock:
            if self._ms is None or time.monotonic() - self._ms_at > STALE_FRAME_S:
                return None
            return round(self._ms, 1)

    def add_viewer(self, delta):
        with self._lock:
            self.viewers += delta


# --------------------------------------------------------------------------- sources


class TestPatternSource:
    """Synthetic frames: a sweeping bar, a crack-like polyline and a frame counter."""

    def __init__(self, width=640, height=480, fps=15.0):
        self.width, self.height, self.interval = width, height, 1.0 / fps
        self._next = time.monotonic()
        self._frame = 0
        rng = np.random.default_rng(7)
        self._noise = rng.integers(90, 140, (height, width), dtype=np.uint8)
        steps = np.cumsum(rng.integers(-6, 7, 40))
        xs = np.linspace(60, width - 60, 40)
        self._crack = np.stack([xs, height // 2 + steps], axis=1).astype(np.int32)

    def read(self):
        delay = self._next - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        self._next = max(self._next + self.interval, time.monotonic())
        self._frame += 1
        frame = cv2.cvtColor(self._noise, cv2.COLOR_GRAY2BGR)
        offset = (self._frame * 4) % self.width
        cv2.line(frame, (offset, 0), (offset, self.height), (230, 230, 230), 3)
        cv2.polylines(frame, [self._crack], False, (30, 30, 30), 3)
        cv2.rectangle(frame, (40, 40), (200, 140), (40, 160, 220), -1)
        cv2.putText(frame, f"TEST {self._frame}", (10, self.height - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        return True, frame

    def release(self):
        pass


def open_source(source, timeout_ms):
    """Open a source; returns an object with read()/release() or None on failure."""
    if source == "test":
        return TestPatternSource()
    if source.isdigit():
        backend = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY
        capture = cv2.VideoCapture(int(source), backend)
    else:
        # Timeouts keep a dead ESP32-CAM from blocking read() forever.
        capture = cv2.VideoCapture(
            source, cv2.CAP_FFMPEG,
            [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, timeout_ms, cv2.CAP_PROP_READ_TIMEOUT_MSEC, timeout_ms],
        )
    if not capture.isOpened():
        capture.release()
        return None
    return capture


def capture_loop(args, frames, meter, stop):
    """Read frames forever, reopening the source with exponential backoff when it drops."""
    backoff = args.min_backoff
    while not stop.is_set():
        capture = open_source(args.source, args.timeout_ms)
        if capture is None:
            print(f"Could not open {args.source!r}; retrying in {backoff:.0f} s")
            stop.wait(backoff)
            backoff = min(backoff * 2, args.max_backoff)
            continue
        print(f"Source {args.source!r} connected")
        failures = 0
        try:
            while not stop.is_set():
                ok, frame = capture.read()
                if not ok or frame is None:
                    failures += 1
                    if failures >= args.max_failures:
                        print("Source stopped delivering frames; reconnecting")
                        break
                    stop.wait(0.05)
                    continue
                failures = 0
                backoff = args.min_backoff  # Healthy again, so the next drop retries fast.
                meter.tick()
                frames.put(frame)
        finally:
            capture.release()  # Frees the single ESP32-CAM client slot.
        meter.reset()
        stop.wait(backoff)


def resize_to_width(frame, width):
    if width and frame.shape[1] > width:
        height = round(frame.shape[0] * width / frame.shape[1])
        return cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
    return frame


# --------------------------------------------------------------------------- pipeline


def process_frame(frame, filter_state):
    """Per-frame processing hook: applies the active filter and records its cost."""
    mode, _, merged = filter_state.get()
    if mode == "none":
        return frame
    started = time.perf_counter()
    output = apply_filter(frame, mode, merged)
    filter_state.record((time.perf_counter() - started) * 1000)
    return output


def compose_loop(frames, scaled, raw_jpegs, meter, args, stop):
    """Resize and encode each frame once for the raw view; the filter never runs here."""
    encode = [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality]
    seq = 0
    while not stop.is_set():
        new_seq, frame = frames.wait_newer(seq, timeout=1.0)
        if new_seq == seq:
            continue
        seq = new_seq
        frame = resize_to_width(frame, args.stream_width)
        scaled.put(frame)
        ok, jpeg = cv2.imencode(".jpg", frame, encode)
        if ok:
            raw_jpegs.put(jpeg.tobytes())
            meter.tick()


def encode_filtered(frame, filter_state, raw_jpeg, quality):
    """Return filtered JPEG bytes; with mode 'none' the raw JPEG is reused as is."""
    if filter_state.get()[0] == "none" and raw_jpeg is not None:
        return raw_jpeg
    ok, jpeg = cv2.imencode(".jpg", process_frame(frame, filter_state), [cv2.IMWRITE_JPEG_QUALITY, quality])
    return jpeg.tobytes() if ok else None


def filter_loop(scaled, raw_jpegs, filtered_jpegs, filter_state, args, stop):
    """Filter the newest frame only while /filtered.mjpg has viewers; slow filters skip frames."""
    seq = 0
    while not stop.is_set():
        new_seq, frame = scaled.wait_newer(seq, timeout=1.0)
        if new_seq == seq or filter_state.viewers <= 0:
            seq = new_seq
            continue
        seq = new_seq
        try:
            jpeg = encode_filtered(frame, filter_state, raw_jpegs.get()[1], args.jpeg_quality)
        except (cv2.error, ValueError, TypeError) as error:
            print(f"Filter failed ({error}); switching to none")
            filter_state.set("none")
            continue
        if jpeg:
            filtered_jpegs.put(jpeg)


# --------------------------------------------------------------------------- MQTT


class VisionMqtt:
    """Receives filter commands and publishes a retained online/offline status."""

    def __init__(self, args, filter_state, health):
        self.filter_topic = f"{args.topic}/filter"
        self.status_topic = f"{args.topic}/status"
        self.filter_state = filter_state
        self.health = health
        self._broker = f"{args.broker}:{args.mqtt_port}"
        client_id = f"camera-stream-{socket.gethostname()}-{uuid.uuid4().hex[:6]}"
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
        self.client.will_set(self.status_topic, json.dumps({"state": "offline"}), qos=1, retain=True)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        filter_state.listeners.append(self.publish_status)
        self.client.connect_async(args.broker, args.mqtt_port, keepalive=30)
        self.client.loop_start()

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            print(f"MQTT connection refused: {reason_code}")
            return
        print(f"MQTT connected to {self._broker}; filter on {self.filter_topic}, status on {self.status_topic}")
        client.subscribe(self.filter_topic, qos=1)  # The retained filter, if any, arrives right away.
        self.publish_status()

    def _on_disconnect(self, client, userdata, flags, reason_code, properties):
        print(f"MQTT disconnected: {reason_code}")

    def _on_message(self, client, userdata, message):
        if not message.payload:
            return  # Empty retained message = topic cleared.
        try:
            mode, params = parse_filter_command(message.payload)
            self.filter_state.set(mode, params)
        except (ValueError, TypeError) as error:
            print(f"Ignored filter command {message.payload[:80]!r}: {error}")

    def publish_status(self):
        if not self.client.is_connected():
            return
        info = self.health()
        status = {
            "state": "online",
            "fps": info["fps"],
            "connected": info["connected"],
            "source": info["source"],
            "filter": info["filter"],
            "filterMs": info["filterMs"],
            "host": socket.gethostname(),
            "ts": int(time.time() * 1000),
        }
        self.client.publish(self.status_topic, json.dumps(status), qos=1, retain=True)

    def close(self):
        if self.client.is_connected():
            self.client.publish(self.status_topic, json.dumps({"state": "offline"}), qos=1, retain=True).wait_for_publish(2)
        self.client.disconnect()
        self.client.loop_stop()


def status_loop(publisher, interval, stop):
    while not stop.wait(interval):
        publisher.publish_status()


# --------------------------------------------------------------------------- HTTP


def make_handler(streams, filtered_snapshot, filter_state, health, stop):
    class StreamHandler(BaseHTTPRequestHandler):
        def do_OPTIONS(self):
            self.send_response(204)
            self._common_headers()
            self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
            self.end_headers()

        def do_GET(self):
            url = urlsplit(self.path)
            if url.path == "/raw.mjpg":
                self._stream(streams["raw"])
            elif url.path == "/filtered.mjpg":
                filter_state.add_viewer(1)
                try:
                    self._stream(streams["filtered"])
                finally:
                    filter_state.add_viewer(-1)
            elif url.path == "/snapshot.jpg":
                view = parse_qs(url.query).get("view", ["raw"])[0]
                if view == "raw":
                    self._jpeg(streams["raw"].get()[1])
                elif view == "filtered":
                    self._jpeg(filtered_snapshot())
                else:
                    self._send(400, "text/plain", b"view must be raw or filtered")
            elif url.path == "/health":
                self._send(200, "application/json", json.dumps(health()).encode())
            else:
                self._send(404, "text/plain", b"Not found")

        def _common_headers(self):
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Expires", "0")

        def _send(self, status, content_type, body):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self._common_headers()
            self.end_headers()
            self.wfile.write(body)

        def _jpeg(self, jpeg):
            if jpeg is None:
                self._send(503, "text/plain", b"No frame yet")
            else:
                self._send(200, "image/jpeg", jpeg)

        def _stream(self, jpegs):
            self.send_response(200)
            self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY.decode()}")
            self._common_headers()
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
            pass  # Keep the console for connection and filter events.

    return StreamHandler


def lan_address():
    """Best-effort LAN IP for the startup message (UDP connect sends no packets)."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        try:
            probe.connect(("8.8.8.8", 80))
            return probe.getsockname()[0]
        except OSError:
            return "localhost"


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--source", default=os.environ.get("CAMERA_URL"),
                        help="ESP32-CAM stream URL, webcam index or 'test' (default: $CAMERA_URL)")
    parser.add_argument("--filter", default="none", choices=MODES,
                        help="Filter used until a retained equipo1/robot/vision/filter message says otherwise")
    parser.add_argument("--http-host", default="0.0.0.0")
    parser.add_argument("--http-port", type=int, default=8000)
    parser.add_argument("--stream-width", type=int, default=640, help="Downscale frames wider than this (0 = keep)")
    parser.add_argument("--jpeg-quality", type=int, default=75)
    parser.add_argument("--timeout-ms", type=int, default=5000, help="Open/read timeout for URL sources")
    parser.add_argument("--min-backoff", type=float, default=1.0, help="First reconnect delay in seconds")
    parser.add_argument("--max-backoff", type=float, default=15.0, help="Largest reconnect delay in seconds")
    parser.add_argument("--max-failures", type=int, default=20, help="Consecutive failed reads before reconnecting")
    parser.add_argument("--broker", default="broker.hivemq.com")
    parser.add_argument("--mqtt-port", type=int, default=1883)
    parser.add_argument("--topic", default="equipo1/robot/vision", help="Prefix for the /filter and /status topics")
    parser.add_argument("--status-interval", type=float, default=5.0, help="Seconds between status refreshes")
    parser.add_argument("--no-mqtt", action="store_true", help="Only serve HTTP; the filter comes from --filter")
    args = parser.parse_args(argv)
    if not args.source:
        parser.error("set --source or the CAMERA_URL environment variable")
    args.source = args.source.strip()
    return args


def main():
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    args = parse_args()
    stop = threading.Event()
    frames, scaled = Latest(), Latest()
    streams = {"raw": Latest(), "filtered": Latest()}
    meters = {"camera": RateMeter(), "output": RateMeter()}
    filter_state = FilterState(args.filter)

    def health():
        mode, params, _ = filter_state.get()
        last = meters["camera"].last
        connected = last is not None and time.monotonic() - last < STALE_FRAME_S
        return {
            "source": args.source,
            "connected": connected,
            "fps": round(meters["camera"].rate, 1) if connected else 0.0,
            "outputFps": round(meters["output"].rate, 1) if connected else 0.0,
            "hasFrame": streams["raw"].get()[1] is not None,
            "filter": mode,
            "params": params,
            "filterMs": filter_state.filter_ms(),
            "filteredViewers": filter_state.viewers,
        }

    def filtered_snapshot():
        """Filter the latest frame on demand, so snapshots work without stream viewers."""
        _, frame = scaled.get()
        if frame is None:
            return None
        return encode_filtered(frame, filter_state, streams["raw"].get()[1], args.jpeg_quality)

    publisher = None if args.no_mqtt else VisionMqtt(args, filter_state, health)

    handler = make_handler(streams, filtered_snapshot, filter_state, health, stop)
    server = ThreadingHTTPServer((args.http_host, args.http_port), handler)
    server.daemon_threads = True
    workers = [
        threading.Thread(target=capture_loop, args=(args, frames, meters["camera"], stop), daemon=True),
        threading.Thread(target=compose_loop, args=(frames, scaled, streams["raw"], meters["output"], args, stop), daemon=True),
        threading.Thread(target=filter_loop, args=(scaled, streams["raw"], streams["filtered"], filter_state, args, stop), daemon=True),
        threading.Thread(target=server.serve_forever, daemon=True),
    ]
    if publisher:
        workers.append(threading.Thread(target=status_loop, args=(publisher, args.status_interval, stop), daemon=True))
    for worker in workers:
        worker.start()

    base = f"http://{lan_address()}:{args.http_port}"
    print(f"Raw:      {base}/raw.mjpg")
    print(f"Filtered: {base}/filtered.mjpg  (filter: {args.filter}, MQTT {args.topic}/filter)")
    print(f"Health:   {base}/health")
    print("Press Ctrl+C to stop")

    try:
        while not stop.wait(0.5):
            pass
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.shutdown()
        server.server_close()
        workers[0].join(timeout=args.timeout_ms / 1000 + 1)  # Let capture leave read() before exit.
        if publisher:
            publisher.close()


if __name__ == "__main__":
    main()
