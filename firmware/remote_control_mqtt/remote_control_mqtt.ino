/*
 * Handheld remote control for the MQTT robot (ESP32 DevKit V1).
 *
 * Hardware
 *   Drive joystick (SparkFun shield): JOY_X=34 (turn), JOY_Y=35 (forward/back)
 *   Buttons (INPUT_PULLUP, pressed = LOW): STOP=26, SPEED-=27, SPEED+=14, PATROL=13
 *   Camera joystick (KY-023): CAM_X=32 (pan), CAM_Y=33 (tilt), CAM_SW=25 (click = center)
 *   Vibration motors: left=19, right=18
 *   Battery divider 100k/100k on GPIO36 (VP): battery V = ADC volts * 2
 *
 * MQTT contract (broker.hivemq.com:1883, all JSON)
 *
 *   equipo1/robot/command      QoS 0, not retained. Published by this remote.
 *     Every 100 ms (10 Hz) while connected, also when idle, so the robot can
 *     run a dead-man timeout (~500 ms without messages = stop):
 *       {"command":"DRIVE","left":-255..255,"right":-255..255,
 *        "x":-255..255,"y":-255..255,"speed":100..255,"seq":n}
 *       left/right are the final differential motor commands (speed applied),
 *       x/y the raw joystick axes (x = turn, y = forward). While the stop latch
 *       is active, left/right are 0.
 *     On the STOP button, immediately:  {"command":"STOP","seq":n}
 *       The remote then latches: DRIVE carries left/right 0 until the drive
 *       joystick has been centered for STOP_RELEASE_CENTER_MS.
 *     On the PATROL button:              {"command":"PATROL","seq":n}
 *     seq is one counter shared by every equipo1/robot/command message.
 *
 *   equipo1/robot/camera       QoS 0, not retained. Published by this remote.
 *     Every 100 ms while the camera joystick is off-center, plus one final
 *     zero message when it returns to center (rates; the robot integrates):
 *       {"pan":-255..255,"tilt":-255..255}
 *     On the camera joystick click:      {"command":"CENTER"}
 *
 *   equipo1/robot/remote/status  QoS 0, retained. Published by this remote.
 *     Every 2 s and whenever speed or the stop latch change:
 *       {"state":"online","speed":100..255,"stopLatched":bool,
 *        "battery":volts|null,"rssi":dBm,"uptimeMs":n}
 *     Last will (retained): {"state":"offline"}
 *
 *   equipo1/robot/telemetry    Subscribed. Haptic alerts:
 *     {"sensor":"ultrasonic_distance","valid":true,"value":<cm>} with
 *     value < OBSTACLE_CM, and {"sensor":"gas","value":<ppm>} with
 *     value >= GAS_ALERT_PPM. The pattern repeats at most every
 *     ALERT_REPEAT_MS while the condition persists.
 *
 * Libraries: PubSubClient (Nick O'Leary), ArduinoJson 7.x.
 * Board: "ESP32 Dev Module". Keep both joysticks released at power-on: the
 * center of every axis is calibrated during setup().
 */

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include <wifi_secrets.h>

// Set to 1 to print joystick/speed values on Serial at 5 Hz.
#ifndef SERIAL_DEBUG
#define SERIAL_DEBUG 0
#endif

// ======================================
// 1. WIFI AND MQTT
// ======================================

const char* MQTT_SERVER = "broker.hivemq.com";
const uint16_t MQTT_PORT = 1883;

const char* TOPIC_COMMAND = "equipo1/robot/command";
const char* TOPIC_CAMERA = "equipo1/robot/camera";
const char* TOPIC_REMOTE_STATUS = "equipo1/robot/remote/status";
const char* TOPIC_TELEMETRY = "equipo1/robot/telemetry";

const char* LWT_PAYLOAD = "{\"state\":\"offline\"}";

const uint32_t WIFI_RETRY_MS = 5000;
const uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
const uint32_t MQTT_RETRY_MS = 3000;
const uint16_t MQTT_SOCKET_TIMEOUT_S = 2;
const uint16_t MQTT_BUFFER_BYTES = 512;

// ======================================
// 2. PINS
// ======================================

const uint8_t JOY_X = 34;       // drive: turn
const uint8_t JOY_Y = 35;       // drive: forward/back
const uint8_t BTN_STOP = 26;    // D6
const uint8_t BTN_SPEED_DOWN = 27;  // D4
const uint8_t BTN_SPEED_UP = 14;    // D5
const uint8_t BTN_PATROL = 13;  // D3

const uint8_t CAM_X = 32;       // camera: pan
const uint8_t CAM_Y = 33;       // camera: tilt
const uint8_t CAM_SW = 25;      // camera: click = center

const uint8_t VIB_LEFT = 19;
const uint8_t VIB_RIGHT = 18;

const uint8_t BATTERY_PIN = 36;  // VP, 100k/100k divider
const bool BATTERY_SENSE_ENABLED = true;
const float BATTERY_DIVIDER_RATIO = 2.0f;
// Below this the divider is considered disconnected and battery is reported as null.
const float BATTERY_MIN_VALID_V = 0.5f;

// ======================================
// 3. JOYSTICK, SPEED AND TIMING
// ======================================

const int ADC_MAX = 4095;
const int DEADZONE = 150;          // ADC counts around the calibrated center
const int CALIBRATION_SAMPLES = 40;

// Flip an axis if the hardware reads it reversed.
const bool INVERT_JOY_X = false;
const bool INVERT_JOY_Y = false;
const bool INVERT_CAM_X = false;
const bool INVERT_CAM_Y = false;

const int SPEED_MIN = 100;
const int SPEED_MAX = 255;
const int SPEED_STEP = 25;
const int SPEED_DEFAULT = 180;

const uint32_t COMMAND_PERIOD_MS = 100;   // 10 Hz equipo1/robot/command and equipo1/robot/camera
const uint32_t STATUS_PERIOD_MS = 2000;
const uint32_t BATTERY_PERIOD_MS = 1000;
const uint32_t DEBUG_PERIOD_MS = 200;     // 5 Hz
const uint32_t DEBOUNCE_MS = 30;
// The stop latch is released after the drive joystick stays centered this long.
const uint32_t STOP_RELEASE_CENTER_MS = 300;

// Haptic alert thresholds, kept equal to the Node-RED dashboard.
const float OBSTACLE_CM = 30.0f;
// Placeholder until the gas sensor is chosen and calibrated.
const float GAS_ALERT_PPM = 400.0f;
const uint32_t ALERT_REPEAT_MS = 1500;
// An alert condition expires if no confirming telemetry arrives for this long.
const uint32_t ALERT_HOLD_MS = 2500;

// ======================================
// 4. STATE
// ======================================

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
char clientId[32];

unsigned long lastWifiAttemptMs = 0;
unsigned long wifiAttemptStartedMs = 0;
unsigned long lastMqttAttemptMs = 0;
bool wifiAttemptActive = false;
bool wasWiFiConnected = false;

int centerJoyX = ADC_MAX / 2;
int centerJoyY = ADC_MAX / 2;
int centerCamX = ADC_MAX / 2;
int centerCamY = ADC_MAX / 2;

// Latest scaled inputs (-255..255) and motor mix.
int joyX = 0, joyY = 0, camPan = 0, camTilt = 0;
int motorLeft = 0, motorRight = 0;

int speed = SPEED_DEFAULT;
uint32_t seq = 0;

bool stopLatched = false;
unsigned long centeredSinceMs = 0;
bool joystickCentered = true;

bool cameraWasActive = false;
float batteryVolts = NAN;

unsigned long lastCommandMs = 0;
unsigned long lastStatusMs = 0;
unsigned long lastBatteryMs = 0;
unsigned long lastDebugMs = 0;
bool statusDirty = true;

// ======================================
// 5. BUTTONS (debounced, press edge)
// ======================================

struct Button {
  uint8_t pin;
  bool stableLevel;      // debounced level (HIGH = released)
  bool lastReading;
  unsigned long lastChangeMs;
};

Button btnStop = {BTN_STOP, HIGH, HIGH, 0};
Button btnSpeedDown = {BTN_SPEED_DOWN, HIGH, HIGH, 0};
Button btnSpeedUp = {BTN_SPEED_UP, HIGH, HIGH, 0};
Button btnPatrol = {BTN_PATROL, HIGH, HIGH, 0};
Button btnCamCenter = {CAM_SW, HIGH, HIGH, 0};

void initButton(Button& b) {
  pinMode(b.pin, INPUT_PULLUP);
  b.lastReading = digitalRead(b.pin);
  b.stableLevel = b.lastReading;
  b.lastChangeMs = millis();
}

// Returns true once per press (stable HIGH -> LOW transition).
bool buttonPressed(Button& b) {
  bool reading = digitalRead(b.pin);
  unsigned long now = millis();

  if (reading != b.lastReading) {
    b.lastReading = reading;
    b.lastChangeMs = now;
    return false;
  }

  if (reading != b.stableLevel && now - b.lastChangeMs >= DEBOUNCE_MS) {
    b.stableLevel = reading;
    return reading == LOW;
  }

  return false;
}

// ======================================
// 6. VIBRATION (non-blocking state machine)
// ======================================

const uint8_t VIB_MASK_LEFT = 0x01;
const uint8_t VIB_MASK_RIGHT = 0x02;
const uint8_t VIB_MASK_BOTH = VIB_MASK_LEFT | VIB_MASK_RIGHT;

struct Vibration {
  uint8_t mask;
  uint8_t pulsesLeft;
  uint16_t onMs;
  uint16_t offMs;
  bool motorOn;
  unsigned long phaseStartedMs;
};

Vibration vib = {0, 0, 0, 0, false, 0};

void writeVibration(uint8_t mask) {
  digitalWrite(VIB_LEFT, (mask & VIB_MASK_LEFT) ? HIGH : LOW);
  digitalWrite(VIB_RIGHT, (mask & VIB_MASK_RIGHT) ? HIGH : LOW);
}

// Starts a pattern of `pulses` pulses, replacing any running pattern.
void startVibration(uint8_t mask, uint8_t pulses, uint16_t onMs, uint16_t offMs) {
  vib.mask = mask;
  vib.pulsesLeft = pulses;
  vib.onMs = onMs;
  vib.offMs = offMs;
  vib.motorOn = true;
  vib.phaseStartedMs = millis();
  writeVibration(mask);
}

bool vibrationBusy() {
  return vib.pulsesLeft > 0;
}

void updateVibration() {
  if (vib.pulsesLeft == 0) return;

  unsigned long elapsed = millis() - vib.phaseStartedMs;

  if (vib.motorOn) {
    if (elapsed < vib.onMs) return;
    writeVibration(0);
    vib.motorOn = false;
    vib.phaseStartedMs = millis();
    vib.pulsesLeft--;
    return;
  }

  // Motor off: wait the gap, then start the next pulse if any remain.
  if (elapsed < vib.offMs) return;
  vib.motorOn = true;
  vib.phaseStartedMs = millis();
  writeVibration(vib.mask);
}

void vibrateStop() { startVibration(VIB_MASK_BOTH, 1, 600, 0); }
void vibratePatrol() { startVibration(VIB_MASK_BOTH, 2, 120, 120); }
void vibrateSpeed(uint8_t side) { startVibration(side, 1, 80, 0); }
void vibrateObstacle() { startVibration(VIB_MASK_BOTH, 3, 80, 80); }
void vibrateGas() { startVibration(VIB_MASK_BOTH, 2, 300, 150); }

// ======================================
// 7. JOYSTICK
// ======================================

int calibrateCenter(uint8_t pin) {
  long sum = 0;
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    sum += analogRead(pin);
    delay(2);  // setup() only
  }
  return (int)(sum / CALIBRATION_SAMPLES);
}

// Maps a raw ADC reading to -255..255 around the calibrated center.
int scaleAxis(int reading, int center, bool invert) {
  int d = reading - center;
  int value;

  if (abs(d) < DEADZONE) {
    value = 0;
  }
  else if (d > 0) {
    int range = ADC_MAX - center;
    value = range > DEADZONE ? map(d, DEADZONE, range, 0, 255) : 255;
  }
  else {
    int range = center;
    value = range > DEADZONE ? -map(-d, DEADZONE, range, 0, 255) : -255;
  }

  value = constrain(value, -255, 255);
  return invert ? -value : value;
}

void readInputs() {
  joyX = scaleAxis(analogRead(JOY_X), centerJoyX, INVERT_JOY_X);
  joyY = scaleAxis(analogRead(JOY_Y), centerJoyY, INVERT_JOY_Y);
  camPan = scaleAxis(analogRead(CAM_X), centerCamX, INVERT_CAM_X);
  camTilt = scaleAxis(analogRead(CAM_Y), centerCamY, INVERT_CAM_Y);

  // Differential mix: x > 0 turns right (left wheel faster).
  long left = constrain(joyY + joyX, -255, 255);
  long right = constrain(joyY - joyX, -255, 255);
  motorLeft = (int)(left * speed / 255);
  motorRight = (int)(right * speed / 255);
}

void updateStopLatch() {
  bool centered = joyX == 0 && joyY == 0;

  if (centered && !joystickCentered) {
    centeredSinceMs = millis();
  }
  joystickCentered = centered;

  if (stopLatched && centered &&
      millis() - centeredSinceMs >= STOP_RELEASE_CENTER_MS) {
    stopLatched = false;
    statusDirty = true;
  }
}

// ======================================
// 8. BATTERY
// ======================================

void readBattery() {
  if (!BATTERY_SENSE_ENABLED) return;
  if (millis() - lastBatteryMs < BATTERY_PERIOD_MS) return;
  lastBatteryMs = millis();

  uint32_t mv = 0;
  for (int i = 0; i < 8; i++) {
    mv += analogReadMilliVolts(BATTERY_PIN);
  }
  float volts = (mv / 8.0f) / 1000.0f * BATTERY_DIVIDER_RATIO;
  batteryVolts = volts >= BATTERY_MIN_VALID_V ? volts : NAN;
}

// ======================================
// 9. MQTT PUBLISHING
// ======================================

bool publishJson(const char* topic, JsonDocument& doc, bool retained) {
  if (!mqtt.connected()) return false;

  char buffer[256];
  size_t n = serializeJson(doc, buffer, sizeof(buffer));
  return mqtt.publish(topic, (const uint8_t*)buffer, n, retained);
}

void publishDrive() {
  JsonDocument doc;
  doc["command"] = "DRIVE";
  doc["left"] = stopLatched ? 0 : motorLeft;
  doc["right"] = stopLatched ? 0 : motorRight;
  doc["x"] = joyX;
  doc["y"] = joyY;
  doc["speed"] = speed;
  doc["seq"] = ++seq;
  publishJson(TOPIC_COMMAND, doc, false);
}

void publishSimpleCommand(const char* command) {
  JsonDocument doc;
  doc["command"] = command;
  doc["seq"] = ++seq;
  publishJson(TOPIC_COMMAND, doc, false);
}

void publishCamera(int pan, int tilt) {
  JsonDocument doc;
  doc["pan"] = pan;
  doc["tilt"] = tilt;
  publishJson(TOPIC_CAMERA, doc, false);
}

void publishCameraCenter() {
  JsonDocument doc;
  doc["command"] = "CENTER";
  publishJson(TOPIC_CAMERA, doc, false);
}

void publishStatus() {
  JsonDocument doc;
  doc["state"] = "online";
  doc["speed"] = speed;
  doc["stopLatched"] = stopLatched;
  if (isnan(batteryVolts)) {
    doc["battery"] = nullptr;
  }
  else {
    doc["battery"] = roundf(batteryVolts * 100.0f) / 100.0f;
  }
  doc["rssi"] = WiFi.RSSI();
  doc["uptimeMs"] = millis();

  if (publishJson(TOPIC_REMOTE_STATUS, doc, true)) {
    statusDirty = false;
    lastStatusMs = millis();
  }
}

// ======================================
// 10. TELEMETRY -> HAPTIC ALERTS
// ======================================

bool obstacleActive = false;
unsigned long obstacleSeenMs = 0;
bool gasActive = false;
unsigned long gasSeenMs = 0;
unsigned long lastAlertMs = 0;
bool lastAlertWasObstacle = false;

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, TOPIC_TELEMETRY) != 0) return;

  JsonDocument doc;
  if (deserializeJson(doc, payload, length) || !doc.is<JsonObject>()) return;

  const char* sensor = doc["sensor"] | "";
  JsonVariant value = doc["value"];
  bool numeric = value.is<float>() || value.is<int>();

  if (strcmp(sensor, "ultrasonic_distance") == 0) {
    // Invalid readings leave the condition as is; ALERT_HOLD_MS expires it.
    if (!(doc["valid"] | false) || !numeric) return;
    obstacleActive = value.as<float>() < OBSTACLE_CM;
    if (obstacleActive) obstacleSeenMs = millis();
  }
  else if (strcmp(sensor, "gas") == 0) {
    if (!(doc["valid"] | true) || !numeric) return;
    gasActive = value.as<float>() >= GAS_ALERT_PPM;
    if (gasActive) gasSeenMs = millis();
  }
}

void updateHapticAlerts() {
  unsigned long now = millis();
  bool obstacle = obstacleActive && now - obstacleSeenMs < ALERT_HOLD_MS;
  bool gas = gasActive && now - gasSeenMs < ALERT_HOLD_MS;

  if (!obstacle && !gas) return;
  if (now - lastAlertMs < ALERT_REPEAT_MS) return;
  // User feedback (buttons) has priority; retry on the next loop.
  if (vibrationBusy()) return;

  // Alternate when both conditions are active.
  bool useObstacle = obstacle && (!gas || !lastAlertWasObstacle);
  if (useObstacle) vibrateObstacle();
  else vibrateGas();

  lastAlertWasObstacle = useObstacle;
  lastAlertMs = now;
}

// ======================================
// 11. BUTTON ACTIONS
// ======================================

void handleButtons() {
  if (buttonPressed(btnStop)) {
    stopLatched = true;
    centeredSinceMs = millis();
    statusDirty = true;
    publishSimpleCommand("STOP");
    vibrateStop();
  }

  if (buttonPressed(btnPatrol)) {
    publishSimpleCommand("PATROL");
    vibratePatrol();
  }

  if (buttonPressed(btnSpeedDown)) {
    int next = max(SPEED_MIN, speed - SPEED_STEP);
    if (next != speed) {
      speed = next;
      statusDirty = true;
    }
    vibrateSpeed(VIB_MASK_LEFT);
  }

  if (buttonPressed(btnSpeedUp)) {
    int next = min(SPEED_MAX, speed + SPEED_STEP);
    if (next != speed) {
      speed = next;
      statusDirty = true;
    }
    vibrateSpeed(VIB_MASK_RIGHT);
  }

  if (buttonPressed(btnCamCenter)) {
    publishCameraCenter();
  }
}

// ======================================
// 12. PERIODIC PUBLISHING
// ======================================

void publishPeriodic() {
  if (!mqtt.connected()) return;

  unsigned long now = millis();

  if (now - lastCommandMs >= COMMAND_PERIOD_MS) {
    lastCommandMs = now;
    publishDrive();

    bool cameraActive = camPan != 0 || camTilt != 0;
    if (cameraActive) {
      publishCamera(camPan, camTilt);
    }
    else if (cameraWasActive) {
      publishCamera(0, 0);  // one final zero on return to center
    }
    cameraWasActive = cameraActive;
  }

  if (statusDirty || now - lastStatusMs >= STATUS_PERIOD_MS) {
    publishStatus();
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
    if (millis() - wifiAttemptStartedMs < WIFI_CONNECT_TIMEOUT_MS) {
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
// 14. MQTT CONNECTION
// ======================================

// mqtt.connect() blocks for up to the socket timeout; it only runs every
// MQTT_RETRY_MS while disconnected, when no commands can be sent anyway.
void connectMqtt() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (mqtt.connected()) return;
  if (millis() - lastMqttAttemptMs < MQTT_RETRY_MS) return;

  lastMqttAttemptMs = millis();

  if (mqtt.connect(clientId, TOPIC_REMOTE_STATUS, 0, true, LWT_PAYLOAD)) {
    Serial.println("MQTT conectado");
    // Disable Nagle: the socket exists only after connect. Sending every 100 ms
    // over an internet RTT, Nagle would hold each small packet until the
    // previous one is ACKed.
    wifiClient.setNoDelay(true);
    mqtt.subscribe(TOPIC_TELEMETRY);
    cameraWasActive = false;
    statusDirty = true;
  }
  else {
    Serial.print("Fallo la conexion MQTT. Codigo: ");
    Serial.println(mqtt.state());
  }
}

// ======================================
// 15. SERIAL DEBUG
// ======================================

void printDebug() {
#if SERIAL_DEBUG
  if (millis() - lastDebugMs < DEBUG_PERIOD_MS) return;
  lastDebugMs = millis();

  Serial.printf(
    "X:%4d Y:%4d | L:%4d R:%4d | vel:%3d | pan:%4d tilt:%4d | stop:%d | bat:%.2f | mqtt:%d\n",
    joyX, joyY, motorLeft, motorRight, speed, camPan, camTilt,
    stopLatched ? 1 : 0, isnan(batteryVolts) ? 0.0f : batteryVolts,
    mqtt.connected() ? 1 : 0
  );
#endif
}

// ======================================
// 16. SETUP
// ======================================

void setup() {
  Serial.begin(115200);

  pinMode(VIB_LEFT, OUTPUT);
  pinMode(VIB_RIGHT, OUTPUT);
  writeVibration(0);

  initButton(btnStop);
  initButton(btnSpeedDown);
  initButton(btnSpeedUp);
  initButton(btnPatrol);
  initButton(btnCamCenter);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // Both joysticks must be released while the centers are measured.
  centerJoyX = calibrateCenter(JOY_X);
  centerJoyY = calibrateCenter(JOY_Y);
  centerCamX = calibrateCenter(CAM_X);
  centerCamY = calibrateCenter(CAM_Y);

  Serial.printf("Centros -> JOY X:%d Y:%d | CAM X:%d Y:%d\n",
    centerJoyX, centerJoyY, centerCamX, centerCamY);

  uint64_t mac = ESP.getEfuseMac();
  snprintf(clientId, sizeof(clientId), "esp32-remote-%04X%08lX",
    (unsigned int)((mac >> 32) & 0xFFFF), (unsigned long)(mac & 0xFFFFFFFF));

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // modem sleep adds 100+ ms latency to incoming MQTT (haptic alerts)

  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);
  mqtt.setBufferSize(MQTT_BUFFER_BYTES);
  mqtt.setSocketTimeout(MQTT_SOCKET_TIMEOUT_S);

  readBattery();

  Serial.println("CONTROL REMOTO INICIADO");
  Serial.println("Esperando WiFi y MQTT...");
}

// ======================================
// 17. MAIN LOOP (no delay())
// ======================================

void loop() {
  readInputs();
  updateStopLatch();
  handleButtons();
  readBattery();

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

  publishPeriodic();
  updateHapticAlerts();
  updateVibration();
  printDebug();
}
