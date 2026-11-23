# Future idea: YOLOv8 crack segmentation

This proof of concept is kept for reference and is not part of the current robot.
The original plan ran inference on the edge, on a Raspberry Pi on the robot. That
plan was dropped. The camera is now an ESP32-CAM, re-streamed by the laptop
(`../camera_stream/`).

## What is here

| File | Purpose |
|---|---|
| `crack_poc.py` | Runs the segmentation model on an image or folder and prints `robot/vision`-style JSON plus median and p95 latency |
| `webcam_poc.py` | Live webcam window with masks and a crack/no-crack banner (`q` quits, `s` saves a frame) |
| `evaluate.py` | Validates both checkpoints on the test split at 640 and 320 px |
| `stream_server.py` | MJPEG server with drawn detections and MQTT detection messages. `camera_stream/` reuses its threading and MQTT structure. |
| `tools/` | Helpers used to inspect the downloaded checkpoints |
| `requirements.txt` | Pinned dependencies (ultralytics, OpenCV, NumPy, paho-mqtt). The CPU PyTorch install is noted inside. |
| `weights/`, `data/`, `results/` | Checkpoints (`yolov8n-crack.pt`, `yolov8s-crack.pt`), crack-seg dataset, and outputs. These are git-ignored. |

## What was measured

- About **45 ms per frame** with `yolov8n-crack` on the laptop CPU (640 px).
- The first `predict` call takes about **12 s**. `stream_server.py` runs a warm-up
  predict on a blank frame at startup to keep that out of the metrics.

## Picking it up again

The easiest route is to run the model on the laptop, inside the
`camera_stream` per-frame hook (`process_frame`). The laptop already receives
every frame. Results could go on `robot/vision` as this POC did.
