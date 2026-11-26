"""Proof of concept: run a crack segmentation model and emit robot/vision JSON."""
import argparse
import json
import time
from pathlib import Path

import numpy as np
from ultralytics import YOLO

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png"}


def build_message(result, elapsed_ms):
    confidences = result.boxes.conf.tolist() if result.boxes is not None else []
    return {
        "sensor": "crack_detector",
        "valid": True,
        "detected": len(confidences) > 0,
        "count": len(confidences),
        "confidence": round(max(confidences), 3) if confidences else None,
        "inferenceMs": round(elapsed_ms, 1),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="Image file or folder")
    parser.add_argument("--weights", default="weights/yolov8n-crack.pt")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--output", type=Path, default=Path("results/predictions"))
    args = parser.parse_args()

    if args.source.is_dir():
        images = sorted(p for p in args.source.iterdir() if p.suffix.lower() in IMAGE_SUFFIXES)
    else:
        images = [args.source]
    args.output.mkdir(parents=True, exist_ok=True)

    model = YOLO(args.weights)
    timings = []
    for image in images:
        started = time.perf_counter()
        result = model.predict(image, imgsz=args.imgsz, conf=args.conf, device="cpu", verbose=False)[0]
        elapsed_ms = (time.perf_counter() - started) * 1000
        timings.append(elapsed_ms)
        result.save(filename=str(args.output / f"{image.stem}.jpg"))
        print(json.dumps({"image": image.name, **build_message(result, elapsed_ms)}))

    if len(timings) > 1:
        # The first call includes model warm-up, so it is left out of the summary.
        steady = timings[1:]
        print(f"images={len(images)} median_ms={np.median(steady):.1f} p95_ms={np.percentile(steady, 95):.1f}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("User interrupted, exiting...")