
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ======================================
// 1. CONFIGURACION WIFI Y MQTT
// ======================================

#include "secrets.h"


const char* MQTT_SERVER = "broker.hivemq.com";
const uint16_t MQTT_PORT = 1883;

// Creación de topicos
const char* TOPIC_CMD = "robot/command";
const char* TOPIC_RESPONSE = "robot/response";
const char* TOPIC_TELEMETRY = "robot/telemetry";

// ======================================
// 2. PINES DEL puente H
// ======================================

// Canal A del L298N: IN1, IN2 y ENA
const uint8_t MOTOR_A_IN1 = 25;
const uint8_t MOTOR_A_IN2 = 26;
const uint8_t MOTOR_A_EN = 27;

// Canal B del L298N: IN3, IN4 y ENB
const uint8_t MOTOR_B_IN1 = 32;
const uint8_t MOTOR_B_IN2 = 33;
const uint8_t MOTOR_B_EN = 14;


const uint8_t ULTRASONIC_TRIG = 18;
const uint8_t ULTRASONIC_ECHO = 19;
const uint32_t ULTRASONIC_INTERVAL_MS = 1000;
const uint32_t ULTRASONIC_TIMEOUT_US = 30000;
const float ULTRASONIC_MIN_CM = 0.0;
const float ULTRASONIC_MAX_CM = 200.0;

const bool MOTOR_A_FORWARD_POLARITY = true;
const bool MOTOR_B_FORWARD_POLARITY = false;

// Velocidades independientes
uint8_t motorASpeed = 180;
uint8_t motorBSpeed = 180;

const uint32_t FORWARD_MS = 2000;
const uint32_t TURN_90_MS = 650;
const uint32_t PAUSE_MS = 500;

const uint32_t COMMAND_TIMEOUT_MS = 3000;

// ======================================
// 5. OBJETOS Y ESTADO
// ======================================

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

bool routineActive = false;
bool manualMotionActive = false;

unsigned long lastCommandMs = 0;
unsigned long lastTelemetryMs = 0;
unsigned long lastWifiAttemptMs = 0;
unsigned long wifiAttemptStartedMs = 0;
unsigned long lastMqttAttemptMs = 0;
bool wifiAttemptActive = false;
bool wasWiFiConnected = false;

const uint32_t WIFI_RETRY_MS = 5000;
const uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
const uint32_t MQTT_RETRY_MS = 3000;

// ======================================
// 6. RUTINA RECTANGULAR
// ======================================

struct RoutineStep {
  const char* command;
  uint32_t durationMs;
};

RoutineStep rectangleRoutine[] = {
  {"FORWARD", FORWARD_MS},
  {"PAUSE", PAUSE_MS},
  {"CW", TURN_90_MS},
  {"PAUSE", PAUSE_MS},

  {"FORWARD", FORWARD_MS},
  {"PAUSE", PAUSE_MS},
  {"CW", TURN_90_MS},
  {"PAUSE", PAUSE_MS},

  {"FORWARD", FORWARD_MS},
  {"PAUSE", PAUSE_MS},
  {"CW", TURN_90_MS},
  {"PAUSE", PAUSE_MS},

  {"FORWARD", FORWARD_MS},
  {"PAUSE", PAUSE_MS},
  {"CW", TURN_90_MS},

  {"STOP", 0}
};

const size_t RECTANGLE_STEP_COUNT =
  sizeof(rectangleRoutine) /
  sizeof(rectangleRoutine[0]);

size_t routineIndex = 0;
unsigned long routineStepStartedMs = 0;

// ======================================
// 7. CONTROL DE MOTORES
// ======================================

void setMotor(
  uint8_t in1,
  uint8_t in2,
  uint8_t en,
  bool forward,
  uint8_t speed
) {
  // Detener PWM antes de invertir sentido
  analogWrite(en, 0);

  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW : HIGH);

  analogWrite(en, speed);
}

void stopMotors() {
  analogWrite(MOTOR_A_EN, 0);
  analogWrite(MOTOR_B_EN, 0);

  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
}

void driveMotorA(bool forward, uint8_t speed) {
  setMotor(
    MOTOR_A_IN1,
    MOTOR_A_IN2,
    MOTOR_A_EN,
    forward,
    speed
  );
}

void driveMotorB(bool forward, uint8_t speed) {
  setMotor(
    MOTOR_B_IN1,
    MOTOR_B_IN2,
    MOTOR_B_EN,
    forward,
    speed
  );
}

// ======================================
// 8. RESPUESTAS Y TELEMETRIA
// ======================================

void publishStatus(
  const String& state,
  const String& detail = ""
) {
  StaticJsonDocument<256> doc;

  doc["ok"] = state != "ERROR";
  doc["state"] = state;
  doc["motorASpeed"] = motorASpeed;
  doc["motorBSpeed"] = motorBSpeed;
  doc["routineActive"] = routineActive;
  doc["uptimeMs"] = millis();

  if (detail.length() > 0) {
    doc["detail"] = detail;
  }

  char buffer[256];
  size_t n = serializeJson(doc, buffer);

  if (mqtt.connected()) {
    mqtt.publish(TOPIC_RESPONSE, buffer, n);
  }

  Serial.println(buffer);
}

void stopRobot(const String& detail = "") {
  routineActive = false;
  manualMotionActive = false;

  stopMotors();
  publishStatus("STOP", detail);
}

// ======================================
// 9. MOVIMIENTOS
// ======================================

void moveForward() {
  driveMotorA(MOTOR_A_FORWARD_POLARITY, motorASpeed);
  driveMotorB(MOTOR_B_FORWARD_POLARITY, motorBSpeed);

  publishStatus("FORWARD");
}

void moveBackward() {
  driveMotorA(!MOTOR_A_FORWARD_POLARITY, motorASpeed);
  driveMotorB(!MOTOR_B_FORWARD_POLARITY, motorBSpeed);

  publishStatus("BACKWARD");
}

void turnClockwise() {
  driveMotorA(MOTOR_A_FORWARD_POLARITY, motorASpeed);
  driveMotorB(!MOTOR_B_FORWARD_POLARITY, motorBSpeed);

  publishStatus("CW");
}

void turnCounterClockwise() {
  driveMotorA(!MOTOR_A_FORWARD_POLARITY, motorASpeed);
  driveMotorB(MOTOR_B_FORWARD_POLARITY, motorBSpeed);

  publishStatus("CCW");
}

// ======================================
// 10. EJECUCION DE COMANDOS
// ======================================

void executeMovementCommand(const String& command) {
  if (command == "FORWARD") {
    moveForward();
  }
  else if (command == "BACKWARD") {
    moveBackward();
  }
  else if (command == "CW") {
    turnClockwise();
  }
  else if (command == "CCW") {
    turnCounterClockwise();
  }
  else if (command == "STOP") {
    stopRobot("manual stop");
  }
  else if (command == "PAUSE") {
    stopMotors();
    publishStatus("PAUSE");
  }
  else {
    stopRobot("Comando desconocido: " + command);
  }
}

// ======================================
// 11. RUTINA DEL RECTANGULO
// ======================================

void startRectangleRoutine() {
  stopMotors();

  routineActive = true;
  manualMotionActive = false;

  routineIndex = 0;
  routineStepStartedMs = millis();

  publishStatus("RECTANGLE", "Empezando rutina");

  executeMovementCommand(
    rectangleRoutine[routineIndex].command
  );
}

void updateRoutine() {
  if (!routineActive) return;

  const RoutineStep& step =
    rectangleRoutine[routineIndex];

  if (millis() - routineStepStartedMs <
      step.durationMs) {
    return;
  }

  routineIndex++;

  if (routineIndex >= RECTANGLE_STEP_COUNT) {
    stopRobot("Rutina terminada");
    return;
  }

  routineStepStartedMs = millis();

  const char* nextCommand =
    rectangleRoutine[routineIndex].command;

  executeMovementCommand(nextCommand);
}

// ======================================
// 12. INTERPRETAR MENSAJES MQTT
// ======================================

void onMqttMessage(
  char* topic,
  byte* payload,
  unsigned int length
) {
  String raw;

  for (unsigned int i = 0; i < length; i++) {
    raw += (char)payload[i];
  }

  raw.trim();

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, raw);

  if (error || !doc.is<JsonObject>()) {
    publishStatus("ERROR", "El comando debe ser un objeto de tipo Json");
    return;
  }

  if (!doc["command"].is<const char*>()) {
    publishStatus("ERROR", "El comando debe ser un string");
    return;
  }

  String command = doc["command"].as<const char*>();
  command.trim();
  command.toUpperCase();

  if (command.length() == 0) {
    publishStatus("ERROR", "Comando faltante");
    return;
  }

  if (doc.containsKey("speed")) {
    int speed = constrain(
      doc["speed"].as<int>(), 0, 255
    );

    motorASpeed = speed;
    motorBSpeed = speed;
  }

  if (doc.containsKey("motorASpeed")) {
    motorASpeed = constrain(
      doc["motorASpeed"].as<int>(), 0, 255
    );
  }

  if (doc.containsKey("motorBSpeed")) {
    motorBSpeed = constrain(
      doc["motorBSpeed"].as<int>(), 0, 255
    );
  }

  lastCommandMs = millis();

  // Cancelar cualquier movimiento anterior
  stopMotors();
  routineActive = false;
  manualMotionActive = false;

  if (command == "RECTANGLE") {
    startRectangleRoutine();
  }
  else if (command == "STOP") {
    stopRobot("manual stop");
  }
  else if (
    command == "FORWARD" ||
    command == "BACKWARD" ||
    command == "CW" ||
    command == "CCW"
  ) {
    manualMotionActive = true;
    executeMovementCommand(command);
  }
  else {
    publishStatus("ERROR", "unknown command: " + command);
  }
}

// ======================================
// 13. WIFI
// ======================================

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    wifiAttemptActive = false;
    return;
  }

  if (wifiAttemptActive) {
    if (millis() - wifiAttemptStartedMs <
        WIFI_CONNECT_TIMEOUT_MS) {
      return;
    }

    Serial.println("Tiempo de espera WiFi agotado. Reintentando...");
    WiFi.disconnect();
    wifiAttemptActive = false;
  }

  if (millis() - lastWifiAttemptMs < WIFI_RETRY_MS) {
    return;
  }

  lastWifiAttemptMs = millis();
  wifiAttemptStartedMs = millis();
  wifiAttemptActive = true;

  Serial.print("Conectando a WiFi: ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

// ======================================
// 14. MQTT
// ======================================

// Inicia la busqueda de la red para poder iniciar el MQTT
void connectMqtt() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (mqtt.connected()) return;

  if (millis() - lastMqttAttemptMs <
      MQTT_RETRY_MS) {
    return;
  }

  lastMqttAttemptMs = millis();

  String clientId = "esp32-robot-" +
    String((uint32_t)ESP.getEfuseMac(), HEX);

  if (mqtt.connect(clientId.c_str())) {
    Serial.println("MQTT connected");

    mqtt.subscribe(TOPIC_CMD);

    publishStatus(
      "ONLINE",
      "MQTT listo"
    );
  }
  else {
    Serial.print("Fallo la conexión MQTT. Código: ");
    Serial.println(mqtt.state());
  }
}

// ======================================
// 15. TELEMETRIA
// ======================================

bool readUltrasonicDistance(float& distanceCm) {
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);

  unsigned long durationUs = pulseIn(
    ULTRASONIC_ECHO,
    HIGH,
    ULTRASONIC_TIMEOUT_US
  );

  if (durationUs == 0) return false;

  distanceCm = durationUs * 0.0343f / 2.0f;
  return distanceCm >= ULTRASONIC_MIN_CM &&
    distanceCm <= ULTRASONIC_MAX_CM;
}

void publishTelemetry() {
  if (!mqtt.connected()) return;

  if (millis() - lastTelemetryMs < ULTRASONIC_INTERVAL_MS) {
    return;
  }

  lastTelemetryMs = millis();

  float distanceCm = 0;
  bool valid = readUltrasonicDistance(distanceCm);

  StaticJsonDocument<256> doc;
  doc["sensor"] = "ultrasonic_distance";
  doc["unit"] = "cm";
  doc["valid"] = valid;
  doc["uptimeMs"] = millis();

  if (valid) {
    doc["value"] = distanceCm;
  }
  else {
    doc["value"] = nullptr;
    doc["reason"] = "Tiempo de espera finalizado o distancia fuera del rango";
  }

  char buffer[256];
  size_t n = serializeJson(doc, buffer);

  mqtt.publish(TOPIC_TELEMETRY, buffer, n);
}

// ======================================
// 16. SETUP
// ======================================

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_A_EN, OUTPUT);

  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  pinMode(MOTOR_B_EN, OUTPUT);

  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
  digitalWrite(ULTRASONIC_TRIG, LOW);

  stopMotors();

  WiFi.mode(WIFI_STA);

  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);

  Serial.println("ROBOT INICIADO");
  Serial.println("Esperando WiFi y MQTT...");
}

// ======================================
// 17. LOOP PRINCIPAL
// ======================================

void loop() {
  // Seguridad: detener ante desconexion
  if (WiFi.status() != WL_CONNECTED ||
      !mqtt.connected()) {
    if (routineActive || manualMotionActive) {
      stopRobot("Conexión perdida!!!");
    }
  }

  connectWiFi();

  if (WiFi.status() == WL_CONNECTED && !wasWiFiConnected) {
    wasWiFiConnected = true;
    Serial.print("WiFi conectado. IP: ");
    Serial.println(WiFi.localIP());
  }
  else if (WiFi.status() != WL_CONNECTED) {
    wasWiFiConnected = false;
  }

  connectMqtt();

  if (mqtt.connected()) {
    mqtt.loop();
  }

  updateRoutine();

  // Parada automatica de movimientos manuales
  if (manualMotionActive &&
      millis() - lastCommandMs >=
      COMMAND_TIMEOUT_MS) {
    stopRobot("Paro...");
  }

  publishTelemetry();
}
