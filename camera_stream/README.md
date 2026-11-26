# camera_stream

Re-streamer del lado de la laptop para la cámara del robot. Lee el stream MJPEG
de la ESP32-CAM una sola vez y lo sirve a cualquier número de navegadores, como el HUD
del teléfono (dashboard de Node-RED) y una pestaña en la laptop. Sirve el video original y un segundo
stream con un filtro de grietas/bordes aplicado.

## Por qué lo hace la laptop

- **La ESP32-CAM atiende a un solo cliente a la vez.** El CameraWebServer de fábrica
  normalmente acepta un único cliente en `/stream`. Si el teléfono se conectara directamente,
  nadie más podría verlo. Aquí la laptop es el único cliente y distribuye el
  video.
- **Cómputo.** Decodificar, filtrar y volver a codificar es económico en una laptop (alrededor de
  20-35 ms por cuadro de 640x480 con cualquier filtro) y demasiado pesado para el ESP32.

## Ejecución

```sh
cd camera_stream
uv sync   # la primera vez: crea .venv a partir de pyproject.toml + uv.lock

# ESP32-CAM (CameraWebServer transmite en el puerto 81)
uv run stream_server.py --source http://192.168.1.60:81/stream
# o con una variable de entorno (PowerShell: $env:CAMERA_URL = "...")
CAMERA_URL=http://192.168.1.60:81/stream uv run stream_server.py

# Webcam de la laptop (índice 0), o un patrón de prueba sintético sin hardware
uv run stream_server.py --source 0
uv run stream_server.py --source test

# Iniciar con un filtro (un mensaje retenido en equipo1/robot/vision/filter lo sobrescribe)
uv run stream_server.py --source test --filter blackhat

# Pruebas
uv run pytest
```

Si la fuente se cae, el servidor sigue en ejecución y se reconecta con backoff
exponencial (de 1 s hasta 15 s, ver `--min-backoff` y `--max-backoff`). Las fuentes URL
usan un timeout de apertura/lectura de 5 s (`--timeout-ms`). Ejecute `--help` para ver todas las opciones.

## Endpoints HTTP (por defecto `0.0.0.0:8000`)

| Ruta | Contenido |
|---|---|
| `/raw.mjpg` | Video en vivo, MJPEG `multipart/x-mixed-replace`. El filtro nunca lo modifica |
| `/filtered.mjpg` | Video en vivo con el filtro activo |
| `/snapshot.jpg?view=raw\|filtered` | Último cuadro (por defecto `raw`; 503 hasta que llegue el primer cuadro) |
| `/health` | JSON: `source`, `connected`, `fps`, `outputFps`, `hasFrame`, `filter`, `params`, `filterMs`, `filteredViewers` |

Cada respuesta envía `Access-Control-Allow-Origin: *` y encabezados sin caché, de modo que
el dashboard en el teléfono pueda incrustar el stream desde la laptop. En el teléfono,
use la IP de la laptop en la red local (el servidor la imprime al iniciar), no `localhost`.
En el flujo de Node-RED, configúrela en el nodo inject "URL del servidor de cámara".
Windows puede solicitar permitir Python a través del firewall para el puerto 8000.

## MQTT

| Tópico | Dirección | Payload |
|---|---|---|
| `equipo1/robot/vision/filter` | suscrito (envíelo retenido) | `canny`, o `{"filter":"canny","params":{"canny_sigma":0.5}}` |
| `equipo1/robot/vision/status` | publicado, retenido, cada 5 s y en cada cambio de filtro | `{"state":"online","fps":15.0,"connected":true,"source":"...","filter":"canny","filterMs":21.5,"host":"...","ts":...}` |
| `equipo1/robot/vision/status` | Last Will (retenido) | `{"state":"offline"}` |

El broker por defecto es `broker.hivemq.com:1883`, el mismo que usan el robot y Node-RED.
Use `--no-mqtt` para servir solo HTTP.

Ambas interfaces de Node-RED publican el filtro por usted. `/ui` tiene un menú desplegable "Filtro" y
un interruptor "Filtrada". `/robot` tiene un selector de filtro y un botón Original/Filtrada.
Este último solo cambia qué stream muestra el navegador.

## Filtros (`filters.py`)

Todos los modos comparten el mismo preprocesamiento: escala de grises, CLAHE y después un filtro
bilateral. La salida pinta los bordes detectados en amarillo sobre el original atenuado.

| Modo | Uso | Pipeline |
|---|---|---|
| `none` | Original | Cuadro sin cambios |
| `canny` | Modo principal | Canny con umbrales automáticos `(1 ± canny_sigma) × median`, limpieza de contornos, cierre |
| `blackhat` | Específico para grietas | Black-hat morfológico (`blackhat_kernel` elíptico), umbral (Otsu con un mínimo, o fijo), limpieza de contornos, cierre |
| `sobel` | Comparación | Magnitud del gradiente, normalizada a 0..255 |
| `laplacian` | Comparación | Laplaciano absoluto después de un desenfoque gaussiano opcional, normalizado |

**Limpieza de contornos.** Este paso descarta los contornos cuyo rectángulo delimitador rotado es
más corto que `min_length` px o menos alargado que `min_aspect`. Elimina el
ruido de textura y conserva las grietas con forma de línea.

**Parámetros.** Todos los parámetros y sus valores por defecto están en `DEFAULT_PARAMS`.

**Modos desconocidos.** Se reemplazan por `none` y registran una advertencia.

**Costo.** El filtro se ejecuta en su propio hilo y solo mientras `/filtered.mjpg` tiene
espectadores. Un snapshot filtrado se calcula bajo demanda. Ni el stream sin filtrar ni
su tasa de cuadros se ven afectados.
