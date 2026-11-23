# camera_stream

Laptop-side re-streamer for the robot camera. It reads the ESP32-CAM MJPEG
stream once and serves it to any number of browsers, such as the phone HUD
(Node-RED dashboard) and a laptop tab.

## Why the laptop does this

- **The ESP32-CAM serves one client at a time.** The stock CameraWebServer
  usually accepts a single `/stream` client. If the phone connected directly,
  nothing else could watch. Here the laptop is the only client and fans the
  video out.
- **Compute.** Decoding and re-encoding (and later image processing) is cheap on
  a laptop and too heavy for the ESP32.

## Run

```sh
cd camera_stream
uv sync   # first time: creates .venv from pyproject.toml + uv.lock

# ESP32-CAM (CameraWebServer streams on port 81)
uv run stream_server.py --source http://192.168.1.60:81/stream
# or with an environment variable (PowerShell: $env:CAMERA_URL = "...")
CAMERA_URL=http://192.168.1.60:81/stream uv run stream_server.py

# Laptop webcam (index 0), or a synthetic test pattern without any hardware
uv run stream_server.py --source 0
uv run stream_server.py --source test
```

If the source drops, the server keeps running and reconnects with exponential
backoff (1 s up to 15 s, see `--min-backoff` and `--max-backoff`). URL sources
use a 5 s open/read timeout (`--timeout-ms`). Run `--help` for every option.

## HTTP endpoints (default `0.0.0.0:8000`)

| Path | Content |
|---|---|
| `/raw.mjpg` | Live video, `multipart/x-mixed-replace` MJPEG |
| `/snapshot.jpg` | Latest frame (503 until the first frame arrives) |
| `/health` | JSON: `source`, `connected`, `fps`, `outputFps`, `hasFrame` |

Every response sends `Access-Control-Allow-Origin: *` and no-cache headers, so
the dashboard on the phone can embed the stream from the laptop. On the phone,
use the laptop's LAN IP (the server prints it at startup), not `localhost`.
In the Node-RED flow, set it in the inject node "URL del servidor de cámara".
Windows may ask to allow Python through the firewall for port 8000.

## MQTT

| Topic | Direction | Payload |
|---|---|---|
| `robot/vision/status` | published, retained, every 5 s | `{"state":"online","fps":15.0,"connected":true,"source":"...","host":"...","ts":...}` |
| `robot/vision/status` | Last Will (retained) | `{"state":"offline"}` |

The default broker is `broker.hivemq.com:1883`, the same one the robot and Node-RED
use. Use `--no-mqtt` to serve HTTP only.

## Extending

`process_frame()` in `stream_server.py` is the single per-frame hook. It runs
once per frame before JPEG encoding and currently returns the frame unchanged.
It is the place to add image processing later, such as edge filters for crack
inspection.
