# Reporte breve — Robot móvil IoT

## 1. Diagrama general del sistema

```text
[Navegador] -> [Node-RED] -> [HiveMQ MQTT] -> [ESP32] -> [L298N] -> [Motores]
                         <-                <-
```

## 2. Comunicación MQTT

- Broker: HiveMQ público (`broker.hivemq.com`).
- Puerto: `1883`.
- Tópico de comandos: `robot/cmd`.
- Tópico de estado: `robot/status`.
- Tópico de telemetría: `robot/telemetry`.

Formato de comando usado:

```json
{"command":"FORWARD","speed":180}
```

Comandos implementados: `FORWARD`, `BACKWARD`, `CW`, `CCW`, `STOP`, `RECTANGLE`.

## 3. Captura de la GUI

Agregar captura del Dashboard en `http://localhost:1880/ui`.

## 4. Rutina automática

La rutina rectangular se inicia con el comando `RECTANGLE` y ejecuta:

1. avanzar por tiempo;
2. girar horario aproximadamente 90°;
3. repetir hasta completar cuatro lados;
4. detenerse.

Parámetros calibrables en el firmware:

- `DEFAULT_SPEED`;
- `FORWARD_MS`;
- `TURN_90_MS`.

## 5. Problema encontrado y solución

Ejemplo para completar:

- Problema: el robot no giraba exactamente 90°.
- Causa: la rutina usa tiempo y los motores no giran idéntico bajo carga.
- Solución: calibramos `TURN_90_MS` y la velocidad PWM hasta obtener un giro aproximado.

## 6. Evidencia

Video corto mostrando:

- control manual desde la GUI;
- mensajes MQTT bidireccionales;
- comando `STOP` funcionando;
- rutina rectangular.
