# Primeros pasos

Cómo ejecutar, detener y usar el sistema del robot.

## Componentes

| Componente | Dónde se ejecuta | Qué hace |
|---|---|---|
| Robot (`firmware/esp32_robot_mqtt`) | ESP32 en el robot | Ejecuta `equipo1/robot/command`, publica `equipo1/robot/response` y `equipo1/robot/telemetry` |
| Control remoto (`firmware/remote_control_mqtt`) | Segundo ESP32 (esquemáticos en `fotos/` local, no versionado) | Lee joysticks y botones, publica `equipo1/robot/command`, `equipo1/robot/camera`, `equipo1/robot/remote/status`; vibra ante alertas |
| ESP32-CAM | En el robot | Sirve la cámara como MJPEG (`http://<cam-ip>:81/stream`) |
| Servidor de stream de cámara (`camera_stream/`) | Laptop | Único cliente de la ESP32-CAM; vuelve a servir el video en el puerto 8000 |
| Dashboard de Node-RED (Docker) | Laptop | HUD para el teléfono: cámara, dirección del control remoto, sensores |
| Mosquitto (Docker) | Laptop | Broker local en 1883/9001 (actualmente sin uso, ver nota) |

> **Broker MQTT:** el dashboard y el firmware usan el broker público
> `broker.hivemq.com:1883`. El contenedor local de Mosquitto se inicia, pero
> ningún flujo lo usa.

## Requisitos previos

- Docker Desktop en ejecución.
- [`uv`](https://docs.astral.sh/uv/) para el servidor de cámara (instala
  Python 3.13 si no está presente).
- Laptop, teléfono y placas ESP32 en la misma red Wi-Fi.

## 1. Iniciar el dashboard (Docker)

Desde la raíz del proyecto:

```bash
docker compose up -d --build   # construir e iniciar Node-RED + Mosquitto
docker compose ps              # verificar que ambos contenedores estén en ejecución
docker compose logs -f nodered # seguir los logs de Node-RED (Ctrl+C para salir)
```

- Dashboard: <http://localhost:1880/ui>
- Página HUD independiente: <http://localhost:1880/robot>
- Editor de Node-RED: <http://localhost:1880>

## 2. Iniciar el servidor de stream de cámara

```bash
cd camera_stream
uv sync   # solo la primera vez: crea .venv a partir de pyproject.toml + uv.lock

# ESP32-CAM
uv run stream_server.py --source http://<cam-ip>:81/stream
# Webcam de la laptop (para pruebas)
uv run stream_server.py --source 0
# Patrón de prueba sintético (no requiere cámara)
uv run stream_server.py --source test
```

### Elegir la fuente

| `--source` | Cámara |
|---|---|
| `http://<cam-ip>:81/stream` | ESP32-CAM en el robot (configuración final) |
| `0` | Webcam integrada de la laptop (`1`, `2`… para otras cámaras conectadas) |
| `test` | Patrón sintético generado por el servidor, sin cámara real |

Si la webcam no abre, cierre cualquier otra aplicación que la esté usando
(Teams, Zoom, Cámara de Windows) o pruebe con el siguiente índice.

### Dónde ver el video

| Qué | URL |
|---|---|
| Stream en vivo | <http://localhost:8000/raw.mjpg> |
| Stream filtrado | <http://localhost:8000/filtered.mjpg> |
| Cuadro individual | <http://localhost:8000/snapshot.jpg> (`?view=filtered` para el filtrado) |
| Estado (fps, conexión, filtro) | <http://localhost:8000/health> |
| Página del robot (incluye video) | <http://localhost:1880/robot> |
| Dashboard (incluye video) | <http://localhost:1880/ui> |

Desde el teléfono, reemplace `localhost` por la IP de la laptop en la red local (ver sección 3).

### Filtros de imagen

Elija el filtro en cualquiera de las dos interfaces: "Filtro" en `/ui`, o el selector en `/robot`.
La elección se publica, con retención, en `equipo1/robot/vision/filter`, de modo que el
servidor de cámara la recupera tras un reinicio. Las opciones son Original, Canny,
Black-hat (específico para grietas), Sobel y Laplaciano.

El interruptor "Filtrada" (`/ui`) y el selector Original/Filtrada (`/robot`)
determinan qué stream se muestra en pantalla. El filtro activo y su costo en ms
aparecen en el estado de la cámara.

Detenga el servidor con `Ctrl+C` en su terminal. Más opciones:
`camera_stream/README.md`.

## 3. Usar el dashboard desde el teléfono

1. Obtenga la IP de la laptop en la red local: `ipconfig` (dirección IPv4, p. ej. `192.168.1.50`).
2. En el teléfono, abra `http://<laptop-ip>:1880/ui`.
3. En el editor de Node-RED, configure el nodo inject `hud-camera-base-url` con
   `http://<laptop-ip>:8000` y haga deploy, para que el teléfono pueda cargar el video.
4. Si el teléfono no puede conectarse, permita los puertos 1880 y 8000 en el
   firewall de Windows.

Vista alternativa: `http://<laptop-ip>:1880/robot` es una página independiente de solo
lectura con los mismos datos (consulta periódicamente `/robot/status`). Carga el video desde
`http://<host desde el que se abrió>:8000` automáticamente, por lo que el paso 3 no es necesario
para ella. Se puede sobrescribir con `?cam=http://<ip>:8000`.

El HUD muestra:

- **Cámara**: stream en vivo y estado del servidor de cámara.
- **Control remoto**: dirección, comandos de motor izquierdo/derecho y velocidad recibidos
  en `equipo1/robot/command` (STOP muestra la tecla de paro en rojo, PATROL muestra
  "Patrulla"), el joystick de cámara y los eventos "Centrar" de `equipo1/robot/camera`,
  el estado del control remoto (en línea/fuera de línea, batería, RSSI, bloqueo de paro) de
  `equipo1/robot/remote/status`, y el estado del robot de `equipo1/robot/response`.
- **Sensores**: distancia ultrasónica, gas (muestra "Sin datos" hasta que el sensor
  publique `{"sensor":"gas", ...}` en `equipo1/robot/telemetry`) y un aviso de alerta.

## 4. Encender el hardware

0. Credenciales Wi-Fi: todos los sketches leen la biblioteca compartida
   `firmware/wifi_secrets`. Configúrela una sola vez como se describe en
   `firmware/wifi_secrets/README.md`.
1. ESP32 del robot: cargue `firmware/esp32_robot_mqtt`.
2. ESP32-CAM: cargue `firmware/esp32_cam_stream` (placa **AI Thinker ESP32-CAM**)
   y anote su IP desde el monitor serie.
3. Control remoto: cárguelo una vez (sección 5) y luego enciéndalo con ambos
   joysticks liberados (sus centros se calibran al encender).

## 5. Firmware del control remoto

1. Las credenciales Wi-Fi provienen de la biblioteca compartida `firmware/wifi_secrets`
   (ver `firmware/wifi_secrets/README.md`).
2. Arduino IDE: placa **ESP32 Dev Module** (núcleo esp32 de Espressif).
3. Bibliotecas: **PubSubClient** (Nick O'Leary) y **ArduinoJson** 7.x.
4. Cargue `firmware/remote_control_mqtt/remote_control_mqtt.ino`.

Pines, retroalimentación háptica y el contrato completo: `firmware/remote_control_mqtt/README.md`.

### Tópicos MQTT

Todos en `broker.hivemq.com:1883`, con payloads JSON.

| Tópico | Publicador | Retenido | Payload |
|---|---|---|---|
| `equipo1/robot/command` | Control remoto, 10 Hz (también en reposo) | No | `{"command":"DRIVE","left":-255..255,"right":-255..255,"x":-255..255,"y":-255..255,"speed":100..255,"seq":n}` |
| `equipo1/robot/command` | Control remoto, botón STOP | No | `{"command":"STOP","seq":n}`, luego DRIVE con `left`/`right` en 0 hasta que el joystick se centre |
| `equipo1/robot/command` | Control remoto, botón PATROL | No | `{"command":"PATROL","seq":n}` |
| `equipo1/robot/camera` | Control remoto, 10 Hz mientras se mueve el joystick de cámara, más un mensaje en cero | No | `{"pan":-255..255,"tilt":-255..255}` (velocidades) |
| `equipo1/robot/camera` | Control remoto, clic del joystick de cámara | No | `{"command":"CENTER"}` |
| `equipo1/robot/remote/status` | Control remoto, cada 2 s y al cambiar | Sí | `{"state":"online","speed":n,"stopLatched":bool,"battery":volts\|null,"rssi":dBm,"uptimeMs":n}`; last will `{"state":"offline"}` |
| `equipo1/robot/response` | Robot | No | Estado del robot |
| `equipo1/robot/telemetry` | Robot | No | `{"sensor":"ultrasonic_distance",...}` / `{"sensor":"gas",...}` |
| `equipo1/robot/vision/filter` | Dashboard | Sí | Filtro de imagen seleccionado |
| `equipo1/robot/vision/status` | Servidor de cámara | — | Estado del servidor de cámara |

El robot debe detenerse si no llega ningún `equipo1/robot/command` durante ~500 ms. El dashboard
aún entiende los comandos anteriores (`FORWARD`, `BACKWARD`, `CW`, `CCW`,
`STOP`, `RECTANGLE`) y nunca publica en `equipo1/robot/command` ni en `equipo1/robot/camera`.

## Detener todo

```bash
# Servidor de cámara: Ctrl+C en su terminal
docker compose stop    # detener los contenedores (los conserva)
docker compose down    # detener y eliminar los contenedores
```

## Ideas futuras

`vision_poc/` conserva una prueba de concepto de segmentación de grietas con YOLOv8. Ver
`vision_poc/FUTURE_IDEAS.md`. El filtrado de bordes (Canny/Sobel/Laplaciano) está planeado
para el servidor de cámara.
