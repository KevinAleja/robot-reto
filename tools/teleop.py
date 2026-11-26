# /// script
# requires-python = ">=3.11"
# dependencies = ["paho-mqtt>=2.0"]
# ///
"""Keyboard teleop for the robot: a stand-in for the handheld remote.

Publishes the same MQTT contract as firmware/remote_control_mqtt:
DRIVE at 10 Hz (so the robot's 500 ms dead-man stays fed), STOP and PATROL.

Keys are latched: press once and the command repeats until you change it.
    w / s     forward / backward
    a / d     spin left / spin right
    q / e     arc forward-left / forward-right
    space, x  stop
    + / -     speed up / down (100..255, step 25)
    p         start patrol
    c         center the camera
    Esc       send STOP and quit (Ctrl+C works too)

Run:  uv run tools/teleop.py    (or: python tools/teleop.py)
"""

import argparse
import json
import sys
import threading
import time
import uuid

import paho.mqtt.client as mqtt

TOPIC_COMMAND = "equipo1/robot/command"
TOPIC_CAMERA = "equipo1/robot/camera"
TOPIC_RESPONSE = "equipo1/robot/response"
TOPIC_TELEMETRY = "equipo1/robot/telemetry"

SEND_PERIOD_S = 0.1
SPEED_MIN, SPEED_MAX, SPEED_STEP = 100, 255, 25

# key -> (x, y) joystick axes in -255..255; x > 0 turns right (remote convention)
DRIVE_KEYS = {
    "w": (0, 255),
    "s": (0, -255),
    "a": (-255, 0),
    "d": (255, 0),
    "q": (-128, 255),
    "e": (128, 255),
    " ": (0, 0),
    "x": (0, 0),
}


def clamp(value, low, high):
    return max(low, min(high, value))


def mix(x, y, speed):
    """Same differential mix as the remote: constrain(y +/- x) * speed / 255."""
    left = clamp(y + x, -255, 255)
    right = clamp(y - x, -255, 255)
    return int(left * speed / 255), int(right * speed / 255)


class KeyReader:
    """Non-blocking single-key reader for Windows and POSIX terminals."""

    def __init__(self):
        self._posix_state = None
        if sys.platform == "win32":
            import msvcrt

            self._msvcrt = msvcrt
        else:
            import termios
            import tty

            fd = sys.stdin.fileno()
            self._posix_state = (fd, termios.tcgetattr(fd))
            tty.setcbreak(fd)

    def read(self):
        if sys.platform == "win32":
            if not self._msvcrt.kbhit():
                return None
            ch = self._msvcrt.getwch()
            if ch in ("\x00", "\xe0"):  # arrow/function key prefix: swallow
                self._msvcrt.getwch()
                return None
            return ch
        import select

        ready, _, _ = select.select([sys.stdin], [], [], 0)
        return sys.stdin.read(1) if ready else None

    def close(self):
        if self._posix_state:
            import termios

            fd, state = self._posix_state
            termios.tcsetattr(fd, termios.TCSADRAIN, state)


class Teleop:
    def __init__(self, host, port, speed):
        self.speed = speed
        self.axes = (0, 0)
        self.seq = 0
        self.lock = threading.Lock()
        self.robot_state = "?"
        self.robot_detail = ""
        self.distance = None
        self.gas = None

        self.client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=f"teleop-{uuid.uuid4().hex[:8]}",
        )
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.on_disconnect = self._on_disconnect
        self.client.reconnect_delay_set(min_delay=1, max_delay=5)
        self.client.connect(host, port, keepalive=10)
        self.client.loop_start()
        # Publishing before CONNACK makes paho drop the session ("Keep alive
        # timeout") and nothing gets through, so wait for the broker first.
        deadline = time.monotonic() + 10
        while not self.client.is_connected():
            if time.monotonic() > deadline:
                self.client.loop_stop()
                sys.exit(f"could not connect to MQTT broker {host}:{port}")
            time.sleep(0.05)

    def _on_disconnect(self, client, userdata, flags, reason_code, properties):
        self.robot_state = f"? (broker disconnected: {reason_code})"

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        client.subscribe([(TOPIC_RESPONSE, 0), (TOPIC_TELEMETRY, 0)])

    def _on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload)
        except ValueError:
            return
        if msg.topic == TOPIC_RESPONSE:
            self.robot_state = data.get("state", "?")
            self.robot_detail = data.get("detail", "")
        elif data.get("sensor") == "ultrasonic_distance":
            self.distance = data.get("value") if data.get("valid") else None
        elif data.get("sensor") == "gas":
            self.gas = data.get("value")

    def publish_command(self, payload):
        if not self.client.is_connected():
            return  # paho reconnects in the background; the dead-man keeps the robot stopped
        with self.lock:
            self.seq += 1
            payload["seq"] = self.seq
            self.client.publish(TOPIC_COMMAND, json.dumps(payload), qos=0)

    def send_drive(self):
        x, y = self.axes
        left, right = mix(x, y, self.speed)
        self.publish_command(
            {"command": "DRIVE", "left": left, "right": right,
             "x": x, "y": y, "speed": self.speed}
        )
        return left, right

    def handle_key(self, key):
        key = key.lower()
        if key in DRIVE_KEYS:
            self.axes = DRIVE_KEYS[key]
            if self.axes == (0, 0):
                self.publish_command({"command": "STOP"})
        elif key in ("+", "="):
            self.speed = clamp(self.speed + SPEED_STEP, SPEED_MIN, SPEED_MAX)
        elif key in ("-", "_"):
            self.speed = clamp(self.speed - SPEED_STEP, SPEED_MIN, SPEED_MAX)
        elif key == "p":
            self.axes = (0, 0)
            self.publish_command({"command": "PATROL"})
        elif key == "c":
            self.client.publish(TOPIC_CAMERA, json.dumps({"command": "CENTER"}))
        elif key == "\x1b":
            return False
        return True

    def status_line(self, left, right):
        dist = f"{self.distance:.0f} cm" if isinstance(self.distance, (int, float)) else "--"
        gas = self.gas if self.gas is not None else "--"
        detail = f" ({self.robot_detail})" if self.robot_detail else ""
        broker = "ok" if self.client.is_connected() else "OFFLINE"
        line = (f"broker={broker} L={left:+4d} R={right:+4d} speed={self.speed:3d} | "
                f"robot={self.robot_state}{detail} | dist={dist} gas={gas}")
        return line[:130].ljust(130)

    def shutdown(self):
        self.axes = (0, 0)
        self.publish_command({"command": "STOP"})
        time.sleep(0.2)
        self.client.loop_stop()
        self.client.disconnect()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="broker.hivemq.com")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--speed", type=int, default=180)
    args = parser.parse_args()

    if not sys.stdin.isatty():
        sys.exit("teleop needs an interactive terminal")

    teleop = Teleop(args.host, args.port, clamp(args.speed, SPEED_MIN, SPEED_MAX))
    keys = KeyReader()
    print(__doc__.split("Run:")[0].strip(), "\n")

    try:
        next_send = time.monotonic()
        running = True
        while running:
            key = keys.read()
            while key is not None and running:
                running = teleop.handle_key(key)
                key = keys.read()
            now = time.monotonic()
            if now >= next_send:
                next_send = now + SEND_PERIOD_S
                left, right = teleop.send_drive()
                print("\r" + teleop.status_line(left, right), end="", flush=True)
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        keys.close()
        teleop.shutdown()
        print("\nSTOP sent. Bye.")


if __name__ == "__main__":
    main()
