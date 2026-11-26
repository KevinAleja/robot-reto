# Firmware del control remoto (`remote_control_mqtt`)

Control remoto de mano para el robot: ESP32 DevKit V1 con un joystick de manejo
(shield de SparkFun), un joystick de cámara (KY-023), cuatro botones y dos motores
de vibración. Publica por MQTT en `broker.hivemq.com:1883`; el robot y el
dashboard de Node-RED se suscriben.

## Carga del firmware

1. Las credenciales Wi-Fi provienen de la biblioteca compartida `firmware/wifi_secrets`
   (ver `../wifi_secrets/README.md`).
2. Arduino IDE: placa **ESP32 Dev Module** (núcleo esp32 de Espressif).
3. Bibliotecas: **PubSubClient** (Nick O'Leary) y **ArduinoJson** 7.x.
4. Mantenga ambos joysticks liberados al encender: los centros de los ejes se calibran
   en `setup()`.
5. Opcional: defina `#define SERIAL_DEBUG 1` para imprimir valores a 5 Hz (115200 baudios).

## Pines

| Función | GPIO |
|---|---|
| Joystick de manejo X (giro) / Y (avance) | 34 / 35 |
| Botones STOP / Velocidad - / Velocidad + / PATROL (pull-up, presionado = LOW) | 26 / 27 / 14 / 13 |
| Joystick de cámara X (pan) / Y (tilt) / clic (centrar) | 32 / 33 / 25 |
| Vibración izquierda / derecha | 19 / 18 |
| Divisor de batería 100k/100k | 36 (VP) |

La dirección de los ejes se puede invertir con `INVERT_JOY_X`, `INVERT_JOY_Y`,
`INVERT_CAM_X`, `INVERT_CAM_Y`.

## Contrato MQTT

| Tópico | Dirección | QoS / retain | Payload |
|---|---|---|---|
| `equipo1/robot/command` | control remoto → robot | 0 / no | `{"command":"DRIVE","left":-255..255,"right":-255..255,"x":-255..255,"y":-255..255,"speed":100..255,"seq":n}` cada 100 ms mientras está conectado, también en reposo |
| `equipo1/robot/command` | control remoto → robot | 0 / no | `{"command":"STOP","seq":n}` al presionar el botón STOP |
| `equipo1/robot/command` | control remoto → robot | 0 / no | `{"command":"PATROL","seq":n}` al presionar el botón PATROL |
| `equipo1/robot/camera` | control remoto → robot | 0 / no | `{"pan":-255..255,"tilt":-255..255}` cada 100 ms mientras el joystick de cámara está fuera del centro, más un mensaje en cero al regresar |
| `equipo1/robot/camera` | control remoto → robot | 0 / no | `{"command":"CENTER"}` al hacer clic en el joystick de cámara |
| `equipo1/robot/remote/status` | control remoto → dashboard | 0 / sí | `{"state":"online","speed":n,"stopLatched":bool,"battery":volts\|null,"rssi":dBm,"uptimeMs":n}` cada 2 s y al cambiar; last will `{"state":"offline"}` |
| `equipo1/robot/telemetry` | robot → control remoto | — | Se lee para las alertas hápticas (ver abajo) |

Notas sobre los campos:

- `left` / `right` son los comandos finales de motor: `constrain(y ± x) * speed / 255`.
  `x` / `y` son los ejes crudos del joystick; `x > 0` gira a la derecha.
- `seq` es un contador único compartido por todos los mensajes de `equipo1/robot/command`; el robot puede
  usarlo para descartar mensajes fuera de orden.
- Pan/tilt son velocidades; el robot las integra en ángulos de servo.

### Comportamiento esperado del robot (para la reescritura del firmware del robot)

- Dead-man: detener los motores si no llega ningún `equipo1/robot/command` durante ~500 ms.
- `STOP`: detenerse de inmediato. Después, el control remoto se bloquea: sigue enviando DRIVE
  con `left`/`right` = 0 hasta que el joystick de manejo haya permanecido centrado durante
  300 ms (`STOP_RELEASE_CENTER_MS`).
- `PATROL`: iniciar la rutina de patrullaje. Los mensajes DRIVE con `left`/`right` = 0
  siguen llegando a 10 Hz durante el patrullaje, por lo que no deben cancelarlo; un
  DRIVE distinto de cero o un STOP sí deben hacerlo.

## Botones y retroalimentación háptica

| Acción | Mensaje | Vibración |
|---|---|---|
| STOP | `STOP` + bloqueo | pulso largo, ambos motores |
| PATROL | `PATROL` | pulso doble, ambos |
| Velocidad - / + (paso 25, rango 100–255, por defecto 180) | nuevo `speed` en DRIVE y en el estado | pulso corto, izquierdo / derecho |
| Clic de cámara | `CENTER` en `equipo1/robot/camera` | — |
| Obstáculo: `ultrasonic_distance` válido y `< OBSTACLE_CM` (30) | — | tres pulsos cortos, como máximo cada 1.5 s |
| Gas: `sensor == "gas"` y `value >= GAS_ALERT_PPM` (400, valor provisional) | — | dos pulsos largos, como máximo cada 1.5 s |

Los umbrales coinciden con los del dashboard de Node-RED. Una alerta se detiene 2.5 s después del último
mensaje de telemetría que la confirma. Todo funciona con `millis()`; `loop()` nunca
llama a `delay()`. La única llamada bloqueante es `mqtt.connect()` (hasta ~2-3 s,
reintentada cada 3 s), y solo mientras MQTT está desconectado.
