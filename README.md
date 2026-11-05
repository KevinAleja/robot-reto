# ESP32 mobile robot with MQTT, JSON, and ultrasonic telemetry

This project controls a differential-drive robot from Node-RED and publishes a real distance measurement from an HC-SR04 ultrasonic sensor.

## Architecture

```text
[Node-RED interface] <--HTTP--> [Browser]
          |
          | MQTT JSON
          v
[HiveMQ public broker] <---- Wi-Fi ----> [ESP32]
                                           |
                         +-----------------+----------------+
                         v                                  v
                    [L298N + motors]                    [HC-SR04]
```

## MQTT contract

All messages exchanged by the robot use valid JSON. The ESP32 does not execute text commands or malformed JSON.

| Topic | Direction | Purpose |
| --- | --- | --- |
| `robot/command` | Node-RED → ESP32 | Robot control commands |
| `robot/response` | ESP32 → Node-RED | Command execution responses and robot state |
| `robot/telemetry` | ESP32 → Node-RED | HC-SR04 distance measurements |

### Command example: `robot/command`

```json
{"command":"FORWARD","speed":180}
```

`command` is required. Allowed commands are `FORWARD`, `BACKWARD`, `CW`, `CCW`, `STOP`, and `RECTANGLE`. The optional `speed`, `motorASpeed`, and `motorBSpeed` values are limited to 0–255.

### Response example: `robot/response`

```json
{"ok":true,"state":"FORWARD","motorASpeed":180,"motorBSpeed":180,"routineActive":false,"uptimeMs":12345}
```

Invalid messages are rejected without moving the robot and return a JSON response such as:

```json
{"ok":false,"state":"ERROR","detail":"command must be a JSON object","uptimeMs":12345}
```

### Telemetry examples: `robot/telemetry`

Valid distance:

```json
{"sensor":"ultrasonic_distance","unit":"cm","valid":true,"uptimeMs":12345,"value":24.5}
```

Unavailable or implausible measurement:

```json
{"sensor":"ultrasonic_distance","unit":"cm","valid":false,"uptimeMs":12345,"value":null,"reason":"timeout or distance outside 2-200 cm"}
```

The ESP32 publishes one measurement every second. This rate is sufficient for a basic obstacle-monitoring demonstration without flooding the broker.

## ESP32 wiring

### L298N motor driver

| ESP32 | L298N |
| --- | --- |
| GPIO 25 | IN1, L298N channel A |
| GPIO 26 | IN2, L298N channel A |
| GPIO 27 | ENA, L298N channel A |
| GPIO 32 | IN3, L298N channel B |
| GPIO 33 | IN4, L298N channel B |
| GPIO 14 | ENB, L298N channel B |
| GND | Common GND |

Use separate power supplies for the motors and ESP32, but connect their grounds together.

### HC-SR04 ultrasonic sensor

| HC-SR04 pin | Connection |
| --- | --- |
| VCC | 5 V |
| GND | Common GND |
| TRIG | ESP32 GPIO 18 |
| ECHO | ESP32 GPIO 19 **through a 5 V → 3.3 V voltage divider** |

> **Important:** HC-SR04 ECHO outputs 5 V. Never connect it directly to an ESP32 GPIO, because ESP32 pins are 3.3 V only. Use a resistor voltage divider (for example, 1 kΩ from ECHO to GPIO 19 and 2 kΩ from GPIO 19 to GND) or a logic-level shifter.

## Start

1. Start Node-RED:

   ```bash
   docker compose up -d
   ```

2. Open the editor at <http://localhost:1880>. If needed, import `nodered/flows.json`.
3. Open the Dashboard at <http://localhost:1880/ui>, or the simple web interface at <http://localhost:1880/robot>.
4. In Arduino IDE, install `PubSubClient` and `ArduinoJson`.
5. Copy `firmware/esp32_robot_mqtt/secrets.example.h` to `secrets.h` in the same folder and set `WIFI_SSID` and `WIFI_PASSWORD`. The broker is configured as `broker.hivemq.com` for testing.
6. Connect the HC-SR04 following the wiring table, including the ECHO voltage divider, and upload the sketch to the ESP32.

## Interface behavior

- Dashboard and `/robot` send JSON commands to `robot/command`.
- Robot responses from `robot/response` appear separately from sensor telemetry.
- The telemetry indicator shows either the current ultrasonic distance in centimetres or a sensor-unavailable warning.

## Manual validation checklist

1. From either interface, send `FORWARD` and then `STOP`. Confirm the robot moves and a JSON response appears on `robot/response`.
2. Subscribe to the three topics with an MQTT client and confirm their payloads are valid JSON:

   ```text
   robot/command
   robot/response
   robot/telemetry
   ```

3. Place a flat object approximately 20–100 cm in front of the sensor. The displayed distance should change reasonably as the object moves.
4. Cover or disconnect the sensor. Confirm that telemetry sets `valid` to `false`, has `value: null`, and provides a `reason`.
5. Publish invalid input, for example `FORWARD` or `{bad json`, to `robot/command`. Confirm the robot does not move and publishes a JSON error response.
6. Record a short video showing a control command, its response, and the distance indicator changing as the object moves.

## Calibration

The rectangular routine is time-based, not encoder-based. Adjust `motorASpeed`, `motorBSpeed`, `FORWARD_MS`, and `TURN_90_MS` in the firmware as needed for the actual robot. The HC-SR04 is treated as valid only between 2 and 200 cm; verify it with objects at known distances before the laboratory demonstration.
