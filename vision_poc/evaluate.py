"""Evaluate both checkpoints on the held-out test split at several sizes."""
from ultralytics import YOLO

for name in ("yolov8n-crack", "yolov8s-crack"):
    for imgsz in (640, 320):
        metrics = YOLO(f"weights/{name}.pt").val(
            data="data/crack-seg.yaml", split="test", imgsz=imgsz, device="cpu",
            verbose=False, plots=False, project="runs", name=f"{name}-{imgsz}", exist_ok=True,
        )
        print(f"RESULT {name} imgsz={imgsz} box_mAP50={metrics.box.map50:.3f} "
              f"mask_mAP50={metrics.seg.map50:.3f} mask_mAP50-95={metrics.seg.map:.3f} "
              f"mask_P={metrics.seg.mp:.3f} mask_R={metrics.seg.mr:.3f}")
