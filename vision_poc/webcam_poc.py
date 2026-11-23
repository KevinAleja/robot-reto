"""Live crack segmentation from a webcam. Press q to quit, s to save the frame."""
import argparse
import json
import time
from pathlib import Path

import cv2
from ultralytics import YOLO

from crack_poc import build_message

WINDOW_NAME = "Crack detector"


def draw_status(frame, message):
    label = "CRACK" if message["detected"] else "no crack"
    color = (0, 0, 255) if message["detected"] else (0, 200, 0)
    confidence = f" {message['confidence']:.2f}" if message["detected"] else ""
    text = f"{label}{confidence} | {message['inferenceMs']:.0f} ms"
    cv2.rectangle(frame, (0, 0), (frame.shape[1], 36), (0, 0, 0), -1)
    cv2.putText(frame, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--weights", default="weights/yolov8n-crack.pt")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--output", type=Path, default=Path("results/webcam"))
    args = parser.parse_args()

    capture = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW)
    if not capture.isOpened():
        raise SystemExit(f"Could not open camera {args.camera}")

    args.output.mkdir(parents=True, exist_ok=True)
    model = YOLO(args.weights)
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                print("Camera read failed")
                break

            started = time.perf_counter()
            result = model.predict(frame, imgsz=args.imgsz, conf=args.conf, device="cpu", verbose=False)[0]
            message = build_message(result, (time.perf_counter() - started) * 1000)

            annotated = result.plot()
            draw_status(annotated, message)
            cv2.imshow(WINDOW_NAME, annotated)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key == ord("s"):
                path = args.output / f"frame_{int(time.time())}.jpg"
                cv2.imwrite(str(path), annotated)
                print(json.dumps({"image": path.name, **message}))
    finally:
        capture.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
