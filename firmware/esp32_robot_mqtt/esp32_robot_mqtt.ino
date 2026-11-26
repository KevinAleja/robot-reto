/*
 * Robot firmware (ESP32 DevKit V1)
 *
 * Driven by the handheld remote (firmware/remote_control_mqtt) over MQTT.
 * See firmware/remote_control_mqtt/README.md for the full contract.
 *
 * Subscribes:
 *   equipo1/robot/command  {"command":"DRIVE","left":-255..255,"right":-255..255,...,"seq":n}  (10 Hz)
 *                  {"command":"STOP","seq":n}
 *                  {"command":"PATROL","seq":n}
 *   equipo1/robot/camera   {"pan":-255..255,"tilt":-255..255}   (rates, integrated into servo angles)
 *                  {"command":"CENTER"}
 * Publishes:
 *   equipo1/robot/response   {"ok","state","detail","left","right","patrol","uptimeMs"} on change + every 2 s
 *   equipo1/robot/telemetry  {"sensor":"ultrasonic_distance","unit":"cm","valid","value"} at 5 Hz
 *                    {"sensor":"gas","unit":"raw","valid","value","voltage"} at 1 Hz
 *
 * Safety:
 *   - Dead-man: motors stop if no equipo1/robot/command arrives for DEADMAN_MS.
 *   - Motors stop when Wi-Fi or MQTT drops.
 *   - Forward motion is blocked while an obstacle is closer than OBSTACLE_STOP_CM.
 *   - Speed changes are ramped to limit current spikes (brownouts).
 *
 * Libraries: PubSubClient, ArduinoJson 7.x, ESP32Servo, wifi_secrets (shared).
 * Everything runs on millis(); loop() never calls delay().
 */

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

#include <wifi_secrets.h>

// ======================================
// 1. NETWORK
// ======================================

const char* MQTT_SERVER = "broker.hivemq.com";
const uint16_t MQTT_PORT = 1883;

const char* TOPIC_CMD = "equipo1/robot/command";
const char* TOPIC_CAMERA = "equipo1/robot/camera";
const char* TOPIC_RESPONSE = "equipo1/robot/response";
const char* TOPIC_TELEMETRY = "equipo1/robot/telemetry";

const uint32_t WIFI_RETRY_MS = 5000;
const uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
const uint32_t MQTT_RETRY_MS = 3000;

// ======================================
// 2. PINS
// ======================================

// Camera servos
const uint8_t PIN_SERVO_PAN = 18;   // servo1
const uint8_t PIN_SERVO_TILT = 19;  // servo2

// Ultrasonic HC-SR04
const uint8_t PIN_TRIG = 23;
const uint8_t PIN_ECHO = 22;

// L298N: channel A (2 motors in parallel) and channel B (2 motors in parallel)
const uint8_t PIN_ENA = 26;
const uint8_t PIN_IN1 = 27;
const uint8_t PIN_IN2 = 14;
const uint8_t PIN_IN3 = 13;
const uint8_t PIN_IN4 = 16;
const uint8_t PIN_ENB = 17;

// MQ gas sensor (ADC1, works with Wi-Fi on). Max 3.3 V on the pin.
const uint8_t PIN_MQ = 34;

// ======================================
// 3. TUNING
// ======================================

// Wiring orientation. If the robot drives backwards, flip INVERT_*.
// If turning is mirrored (joystick right turns left), flip LEFT_IS_CHANNEL_A.
const bool LEFT_IS_CHANNEL_A = true;
const bool INVERT_CHANNEL_A = false;
const bool INVERT_CHANNEL_B = false;

// Commands with |value| <= DEADBAND are treated as 0. Above it, PWM starts at
// MIN_PWM so the motors do not hum without moving at low joystick deflection.
const int DRIVE_DEADBAND = 12;
const int MIN_PWM = 90;

// Ramp: max PWM change per MOTOR_UPDATE_MS (0 -> 255 in ~60 ms).
const uint32_t MOTOR_UPDATE_MS = 10;
const int RAMP_STEP = 45;

// Dead-man: stop if the remote goes silent (it sends DRIVE at 10 Hz).
const uint32_t DEADMAN_MS = 500;
// After this much silence, any seq is accepted (remote rebooted).
const uint32_t SEQ_RESET_GAP_MS = 1000;

// Obstacle guard
const bool OBSTACLE_GUARD = true;
const float OBSTACLE_STOP_CM = 20.0f;
const uint32_t DISTANCE_FRESH_MS = 500;

// Ultrasonic
const uint32_t ULTRASONIC_INTERVAL_MS = 200;
const uint32_t ULTRASONIC_TIMEOUT_US = 12000;  // ~2 m, keeps the blocking read short
const float ULTRASONIC_MAX_CM = 200.0f;

// Gas
const uint32_t GAS_INTERVAL_MS = 1000;
const int GAS_SAMPLES = 8;

// Camera servos
const uint32_t CAMERA_UPDATE_MS = 20;
const uint32_t CAMERA_RATE_TIMEOUT_MS = 300;  // rate falls to 0 if messages stop
const float CAMERA_MAX_DEG_PER_S = 90.0f;     // at full joystick deflection
const int CAMERA_RATE_DEADBAND = 15;
const bool INVERT_PAN = false;
const bool INVERT_TILT = false;
const float PAN_MIN = 0, PAN_MAX = 180, PAN_CENTER = 90;
const float TILT_MIN = 30, TILT_MAX = 150, TILT_CENTER = 90;

// Status heartbeat on equipo1/robot/response
const uint32_t STATUS_HEARTBEAT_MS = 2000;

// ======================================
// 4. PATROL ROUTINE (rectangle)
// ======================================

struct PatrolStep {
  int left;
  int right;
  uint32_t durationMs;
};

const int PATROL_SPEED = 180;
const uint32_t PATROL_FORWARD_MS = 2000;
const uint32_t PATROL_TURN_MS = 650;
const uint32_t PATROL_PAUSE_MS = 400;

const PatrolStep PATROL_STEPS[] = {
  { PATROL_SPEED,  PATROL_SPEED, PATROL_FORWARD_MS},
  { 0,             0,            PATROL_PAUSE_MS},
  { PATROL_SPEED, -PATROL_SPEED, PATROL_TURN_MS},
  { 0,             0,            PATROL_PAUSE_MS},
};
const size_t PATROL_STEP_COUNT = sizeof(PATROL_STEPS) / sizeof(PATROL_STEPS[0]);
const int PATROL_LAPS = 4;  // 4 sides = one rectangle

// ======================================
// 5. STATE
// ======================================

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
Servo servoPan;
Servo servoTilt;

// Drive commands (-255..255): requested by the remote / patrol
int targetLeft = 0;
int targetRight = 0;
// Ramped values actually applied
int appliedLeft = 0;
int appliedRight = 0;
bool obstacleBlocked = false;

unsigned long lastCommandMs = 0;
bool commandSeen = false;
uint32_t lastSeq = 0;
bool haveSeq = false;

bool patrolActive = false;
size_t patrolIndex = 0;
int patrolLap = 0;
unsigned long patrolStepStartedMs = 0;

float distanceCm = 0;
bool distanceValid = false;
unsigned long distanceAtMs = 0;

float panAngle = PAN_CENTER;
float tiltAngle = TILT_CENTER;
int panRate = 0;
int tiltRate = 0;
unsigned long lastCameraMsgMs = 0;
int writtenPan = -1;
int writtenTilt = -1;

String lastPublishedState = "";
unsigned long lastStatusMs = 0;

unsigned long lastMotorUpdateMs = 0;
unsigned long lastCameraUpdateMs = 0;
unsigned long lastUltrasonicMs = 0;
unsigned long lastGasMs = 0;

unsigned long lastWifiAttemptMs = 0;
unsigned long wifiAttemptStartedMs = 0;
unsigned long lastMqttAttemptMs = 0;
bool wifiAttemptActive = false;
bool wasWiFiConnected = false;

// ======================================
// 6. MOTORS
// ======================================

int shapePwm(int magnitude) {
  if (magnitude <= DRIVE_DEADBAND) return 0;
  return map(magnitude, DRIVE_DEADBAND + 1, 255, MIN_PWM, 255);
}

void driveChannel(uint8_t en, uint8_t in1, uint8_t in2, bool invert, int command) {
  int pwm = shapePwm(abs(command));

  if (pwm == 0) {
    analogWrite(en, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    return;
  }

  bool forward = (command > 0) != invert;
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW : HIGH);
  analogWrite(en, pwm);
}

void applyMotors(int left, int right) {
  int a = LEFT_IS_CHANNEL_A ? left : right;
  int b = LEFT_IS_CHANNEL_A ? right : left;
  driveChannel(PIN_ENA, PIN_IN1, PIN_IN2, INVERT_CHANNEL_A, a);
  driveChannel(PIN_ENB, PIN_IN3, PIN_IN4, INVERT_CHANNEL_B, b);
}

// Immediate stop, no ramp.
void hardStopMotors() {
  targetLeft = targetRight = 0;
  appliedLeft = appliedRight = 0;
  applyMotors(0, 0);
}

int rampToward(int current, int target) {
  int delta = constrain(target - current, -RAMP_STEP, RAMP_STEP);
  return current + delta;
}

bool obstacleAhead() {
  return OBSTACLE_GUARD &&
    distanceValid &&
    millis() - distanceAtMs < DISTANCE_FRESH_MS &&
    distanceCm < OBSTACLE_STOP_CM;
}

void updateMotors() {
  if (millis() - lastMotorUpdateMs < MOTOR_UPDATE_MS) return;
  lastMotorUpdateMs = millis();

  int left = targetLeft;
  int right = targetRight;

  // Block net-forward motion near an obstacle; spinning and reversing stay allowed.
  obstacleBlocked = obstacleAhead() && (left + right) > 0;
  if (obstacleBlocked) {
    left = 0;
    right = 0;
  }

  int newLeft = rampToward(appliedLeft, left);
  int newRight = rampToward(appliedRight, right);

  if (newLeft != appliedLeft || newRight != appliedRight) {
    appliedLeft = newLeft;
    appliedRight = newRight;
    applyMotors(appliedLeft, appliedRight);
  }
}

// ======================================
// 7. STATUS
// ======================================

void publishStatus(const char* state, const char* detail = nullptr) {
  JsonDocument doc;
  doc["ok"] = strcmp(state, "ERROR") != 0;
  doc["state"] = state;
  if (detail) doc["detail"] = detail;
  doc["left"] = targetLeft;
  doc["right"] = targetRight;
  doc["patrol"] = patrolActive;
  doc["uptimeMs"] = millis();

  char buffer[256];
  size_t n = serializeJson(doc, buffer, sizeof(buffer));

  if (mqtt.connected()) {
    mqtt.publish(TOPIC_RESPONSE, (const uint8_t*)buffer, n, false);
  }

  lastPublishedState = state;
  lastStatusMs = millis();
  Serial.println(buffer);
}

const char* currentState() {
  if (obstacleBlocked) return "OBSTACLE";
  if (patrolActive) return "PATROL";
  if (targetLeft != 0 || targetRight != 0) return "DRIVE";
  return "IDLE";
}

// Publish when the state changes, plus a slow heartbeat.
void updateStatus() {
  const char* state = currentState();
  bool changed = lastPublishedState != state;
  bool heartbeat = millis() - lastStatusMs >= STATUS_HEARTBEAT_MS;
  if (changed || heartbeat) {
    publishStatus(state);
  }
}

void stopRobot(const char* state, const char* detail) {
  patrolActive = false;
  hardStopMotors();
  publishStatus(state, detail);
}

// ======================================
// 8. PATROL
// ======================================

void startPatrolStep(size_t index) {
  patrolIndex = index;
  patrolStepStartedMs = millis();
  targetLeft = PATROL_STEPS[index].left;
  targetRight = PATROL_STEPS[index].right;
}

void startPatrol() {
  patrolActive = true;
  patrolLap = 0;
  startPatrolStep(0);
  publishStatus("PATROL", "Patrol started");
}

void updatePatrol() {
  if (!patrolActive) return;

  const PatrolStep& step = PATROL_STEPS[patrolIndex];
  bool forwardStep = step.left > 0 && step.right > 0;
  bool timeUp = millis() - patrolStepStartedMs >= step.durationMs;

  // Cut a forward leg short when an obstacle shows up: go straight to the turn.
  if (!timeUp && !(forwardStep && obstacleAhead())) return;

  size_t next = patrolIndex + 1;
  if (next >= PATROL_STEP_COUNT) {
    next = 0;
    patrolLap++;
    if (patrolLap >= PATROL_LAPS) {
      stopRobot("IDLE", "Patrol finished");
      return;
    }
  }
  startPatrolStep(next);
}

// ======================================
// 9. MQTT MESSAGES
// ======================================

// Returns false if the message is older than the last one accepted.
bool acceptSeq(JsonDocument& doc, unsigned long now) {
  if (!doc["seq"].is<uint32_t>()) return true;

  uint32_t seq = doc["seq"].as<uint32_t>();
  bool longGap = !commandSeen || now - lastCommandMs > SEQ_RESET_GAP_MS;

  if (haveSeq && !longGap && seq <= lastSeq) return false;

  lastSeq = seq;
  haveSeq = true;
  return true;
}

void handleCommand(JsonDocument& doc) {
  if (!doc["command"].is<const char*>()) {
    publishStatus("ERROR", "Missing \"command\" string");
    return;
  }

  unsigned long now = millis();
  if (!acceptSeq(doc, now)) return;

  lastCommandMs = now;
  commandSeen = true;

  String command = doc["command"].as<const char*>();
  command.toUpperCase();

  if (command == "DRIVE") {
    int left = constrain(doc["left"] | 0, -255, 255);
    int right = constrain(doc["right"] | 0, -255, 255);
    bool idle = abs(left) <= DRIVE_DEADBAND && abs(right) <= DRIVE_DEADBAND;

    if (patrolActive) {
      if (idle) return;  // centered joystick keeps the patrol running
      patrolActive = false;
      publishStatus("DRIVE", "Patrol cancelled by joystick");
    }

    targetLeft = idle ? 0 : left;
    targetRight = idle ? 0 : right;
  }
  else if (command == "STOP") {
    stopRobot("STOP", "Stop button");
  }
  else if (command == "PATROL") {
    if (!patrolActive) startPatrol();
  }
  else {
    String detail = "Unknown command: " + command;
    publishStatus("ERROR", detail.c_str());
  }
}

void handleCamera(JsonDocument& doc) {
  lastCameraMsgMs = millis();

  if (doc["command"].is<const char*>()) {
    String command = doc["command"].as<const char*>();
    command.toUpperCase();
    if (command == "CENTER") {
      panRate = tiltRate = 0;
      panAngle = PAN_CENTER;
      tiltAngle = TILT_CENTER;
    }
    return;
  }

  int pan = constrain(doc["pan"] | 0, -255, 255);
  int tilt = constrain(doc["tilt"] | 0, -255, 255);
  panRate = abs(pan) <= CAMERA_RATE_DEADBAND ? 0 : pan;
  tiltRate = abs(tilt) <= CAMERA_RATE_DEADBAND ? 0 : tilt;
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload, length);

  if (error || !doc.is<JsonObject>()) {
    publishStatus("ERROR", "Payload must be a JSON object");
    return;
  }

  if (strcmp(topic, TOPIC_CMD) == 0) {
    handleCommand(doc);
  }
  else if (strcmp(topic, TOPIC_CAMERA) == 0) {
    handleCamera(doc);
  }
}

// ======================================
// 10. CAMERA SERVOS
// ======================================

void writeServos() {
  int pan = (int)roundf(panAngle);
  int tilt = (int)roundf(tiltAngle);
  if (pan != writtenPan) {
    servoPan.write(pan);
    writtenPan = pan;
  }
  if (tilt != writtenTilt) {
    servoTilt.write(tilt);
    writtenTilt = tilt;
  }
}

void updateCamera() {
  unsigned long now = millis();
  unsigned long dt = now - lastCameraUpdateMs;
  if (dt < CAMERA_UPDATE_MS) return;
  lastCameraUpdateMs = now;

  if (now - lastCameraMsgMs > CAMERA_RATE_TIMEOUT_MS) {
    panRate = tiltRate = 0;
  }

  float seconds = dt / 1000.0f;
  float panStep = panRate / 255.0f * CAMERA_MAX_DEG_PER_S * seconds;
  float tiltStep = tiltRate / 255.0f * CAMERA_MAX_DEG_PER_S * seconds;

  panAngle = constrain(panAngle + (INVERT_PAN ? -panStep : panStep), PAN_MIN, PAN_MAX);
  tiltAngle = constrain(tiltAngle + (INVERT_TILT ? -tiltStep : tiltStep), TILT_MIN, TILT_MAX);

  writeServos();
}

// ======================================
// 11. SENSORS / TELEMETRY
// ======================================

bool readUltrasonic(float& cm) {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long durationUs = pulseIn(PIN_ECHO, HIGH, ULTRASONIC_TIMEOUT_US);
  if (durationUs == 0) return false;

  cm = durationUs * 0.0343f / 2.0f;
  return cm > 0 && cm <= ULTRASONIC_MAX_CM;
}

void publishJson(const char* topic, JsonDocument& doc) {
  if (!mqtt.connected()) return;
  char buffer[256];
  size_t n = serializeJson(doc, buffer, sizeof(buffer));
  mqtt.publish(topic, (const uint8_t*)buffer, n, false);
}

void updateUltrasonic() {
  if (millis() - lastUltrasonicMs < ULTRASONIC_INTERVAL_MS) return;
  lastUltrasonicMs = millis();

  float cm = 0;
  bool valid = readUltrasonic(cm);
  distanceValid = valid;
  if (valid) distanceCm = cm;
  distanceAtMs = millis();

  JsonDocument doc;
  doc["sensor"] = "ultrasonic_distance";
  doc["unit"] = "cm";
  doc["valid"] = valid;
  if (valid) doc["value"] = roundf(cm * 10) / 10;
  else doc["value"] = nullptr;
  doc["uptimeMs"] = millis();
  publishJson(TOPIC_TELEMETRY, doc);
}

void updateGas() {
  if (millis() - lastGasMs < GAS_INTERVAL_MS) return;
  lastGasMs = millis();

  uint32_t sum = 0;
  for (int i = 0; i < GAS_SAMPLES; i++) sum += analogRead(PIN_MQ);
  int raw = sum / GAS_SAMPLES;

  // Uncalibrated: raw ADC (0-4095). Replace with ppm once the sensor is calibrated.
  JsonDocument doc;
  doc["sensor"] = "gas";
  doc["unit"] = "raw";
  doc["valid"] = true;
  doc["value"] = raw;
  doc["voltage"] = roundf(raw / 4095.0f * 3.3f * 100) / 100;
  doc["uptimeMs"] = millis();
  publishJson(TOPIC_TELEMETRY, doc);
}

// ======================================
// 12. WIFI / MQTT CONNECTION
// ======================================

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    wifiAttemptActive = false;
    return;
  }

  if (wifiAttemptActive) {
    if (millis() - wifiAttemptStartedMs < WIFI_CONNECT_TIMEOUT_MS) return;
    Serial.println("Wi-Fi timeout, retrying...");
    WiFi.disconnect();
    wifiAttemptActive = false;
  }

  if (millis() - lastWifiAttemptMs < WIFI_RETRY_MS) return;

  lastWifiAttemptMs = millis();
  wifiAttemptStartedMs = millis();
  wifiAttemptActive = true;

  Serial.print("Connecting to Wi-Fi: ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectMqtt() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (mqtt.connected()) return;
  if (millis() - lastMqttAttemptMs < MQTT_RETRY_MS) return;
  lastMqttAttemptMs = millis();

  String clientId = "esp32-robot-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  const char* willPayload = "{\"ok\":false,\"state\":\"OFFLINE\"}";

  if (mqtt.connect(clientId.c_str(), nullptr, nullptr, TOPIC_RESPONSE, 0, false, willPayload)) {
    Serial.println("MQTT connected");
    // Disable Nagle: the socket exists only after connect. Sending every 100 ms
    // over an internet RTT, Nagle would hold each small packet until the
    // previous one is ACKed.
    wifiClient.setNoDelay(true);
    mqtt.subscribe(TOPIC_CMD, 0);
    mqtt.subscribe(TOPIC_CAMERA, 0);
    haveSeq = false;
    publishStatus("ONLINE", "MQTT ready");
  }
  else {
    Serial.print("MQTT connect failed, state ");
    Serial.println(mqtt.state());
  }
}

// ======================================
// 13. SETUP
// ======================================

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== ROBOT STARTING ===");

  // Servos first so they take their PWM timers before the motor PWM.
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servoPan.setPeriodHertz(50);
  servoTilt.setPeriodHertz(50);
  servoPan.attach(PIN_SERVO_PAN, 500, 2400);
  servoTilt.attach(PIN_SERVO_TILT, 500, 2400);
  writeServos();

  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  hardStopMotors();

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  analogReadResolution(12);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // modem sleep adds 100+ ms latency to incoming MQTT

  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);
  mqtt.setKeepAlive(5);
  mqtt.setSocketTimeout(3);

  Serial.println("Waiting for Wi-Fi and MQTT...");
}

// ======================================
// 14. LOOP
// ======================================

void loop() {
  bool online = WiFi.status() == WL_CONNECTED && mqtt.connected();
  bool moving = patrolActive || targetLeft != 0 || targetRight != 0 ||
    appliedLeft != 0 || appliedRight != 0;

  if (!online && moving) {
    stopRobot("STOP", "Connection lost");
  }

  connectWiFi();
  if (WiFi.status() == WL_CONNECTED && !wasWiFiConnected) {
    wasWiFiConnected = true;
    Serial.print("Wi-Fi connected. IP: ");
    Serial.println(WiFi.localIP());
  }
  else if (WiFi.status() != WL_CONNECTED) {
    wasWiFiConnected = false;
  }

  connectMqtt();
  if (mqtt.connected()) mqtt.loop();

  if (moving && commandSeen && millis() - lastCommandMs > DEADMAN_MS) {
    stopRobot("STOP", "Dead-man: remote silent");
  }

  updatePatrol();
  updateMotors();
  updateCamera();
  updateUltrasonic();
  updateGas();
  updateStatus();
}
