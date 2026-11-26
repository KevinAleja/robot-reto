# Idea futura: segmentación de grietas con YOLOv8

Esta prueba de concepto se conserva como referencia y no forma parte del robot actual.
El plan original ejecutaba la inferencia en el borde, en una Raspberry Pi montada en el robot. Ese
plan se descartó. La cámara ahora es una ESP32-CAM, cuyo stream retransmite la laptop
(`../camera_stream/`).

## Qué contiene

| Archivo | Propósito |
|---|---|
| `crack_poc.py` | Ejecuta el modelo de segmentación sobre una imagen o carpeta e imprime JSON con el formato de `robot/vision`, además de la latencia mediana y p95 |
| `webcam_poc.py` | Ventana en vivo de la webcam con máscaras y un aviso de grieta/sin grieta (`q` sale, `s` guarda un cuadro) |
| `evaluate.py` | Valida ambos checkpoints sobre el conjunto de prueba a 640 y 320 px |
| `stream_server.py` | Servidor MJPEG con detecciones dibujadas y mensajes MQTT de detección. `camera_stream/` reutiliza su estructura de hilos y MQTT. |
| `tools/` | Utilidades usadas para inspeccionar los checkpoints descargados |
| `requirements.txt` | Dependencias con versión fija (ultralytics, OpenCV, NumPy, paho-mqtt). La instalación de PyTorch para CPU se indica dentro. |
| `weights/`, `data/`, `results/` | Checkpoints (`yolov8n-crack.pt`, `yolov8s-crack.pt`), dataset crack-seg y resultados. git los ignora. |

## Qué se midió

- Alrededor de **45 ms por cuadro** con `yolov8n-crack` en la CPU de la laptop (640 px).
- La primera llamada a `predict` tarda alrededor de **12 s**. `stream_server.py` ejecuta un predict de
  calentamiento sobre un cuadro en blanco al iniciar para excluir ese tiempo de las métricas.

## Retomar la idea

La ruta más sencilla es ejecutar el modelo en la laptop, dentro del hook por cuadro de
`camera_stream` (`process_frame`). La laptop ya recibe
cada cuadro. Los resultados podrían publicarse en `robot/vision`, como lo hacía esta POC.
