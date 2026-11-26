// ESP32-CAM (AI Thinker) MJPEG streamer for camera_stream/stream_server.py.
//
// Endpoints (same layout as the stock CameraWebServer example):
//   http://<ip>:81/stream   MJPEG stream (multipart/x-mixed-replace)
//   http://<ip>/capture     single JPEG frame
//   http://<ip>/            minimal page that shows the stream
//
// Board: "AI Thinker ESP32-CAM", PSRAM enabled. Upload at 115200 through the
// ESP32-CAM-MB base. The IP address is printed on the serial monitor at boot.

#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include <wifi_secrets.h>

// AI Thinker ESP32-CAM pin map
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#define FLASH_LED_GPIO_NUM 4

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static httpd_handle_t webHttpd = nullptr;
static httpd_handle_t streamHttpd = nullptr;

static bool initCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  // 320x240: about 4x lighter frames than VGA, so the weak ESP32-CAM WiFi link
  // keeps a live frame rate. Use FRAMESIZE_VGA (640x480) on a strong signal.
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;           // 0-63, lower is better quality
  config.grab_mode = CAMERA_GRAB_LATEST;

  if (psramFound()) {
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    Serial.println("PSRAM not found, using a single DRAM frame buffer");
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.frame_size = FRAMESIZE_QVGA;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }
  return true;
}

static esp_err_t streamHandler(httpd_req_t* req) {
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  char partHeader[64];
  while (true) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Frame capture failed");
      res = ESP_FAIL;
      break;
    }
    size_t headerLen = snprintf(partHeader, sizeof(partHeader), STREAM_PART, fb->len);
    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, partHeader, headerLen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;  // client disconnected
  }
  return res;
}

static esp_err_t captureHandler(httpd_req_t* req) {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  esp_err_t res = httpd_resp_send(req, (const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return res;
}

static esp_err_t indexHandler(httpd_req_t* req) {
  static const char page[] =
      "<!doctype html><html><head><meta name=viewport content='width=device-width'>"
      "<title>ESP32-CAM</title></head><body style='margin:0;background:#111'>"
      "<img id=s style='width:100%'>"
      "<script>s.src='http://'+location.hostname+':81/stream'</script>"
      "</body></html>";
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
}

static void startServers() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  httpd_uri_t indexUri = {"/", HTTP_GET, indexHandler, nullptr};
  httpd_uri_t captureUri = {"/capture", HTTP_GET, captureHandler, nullptr};
  if (httpd_start(&webHttpd, &config) == ESP_OK) {
    httpd_register_uri_handler(webHttpd, &indexUri);
    httpd_register_uri_handler(webHttpd, &captureUri);
  }

  // The stream blocks its handler, so it gets its own server on port 81.
  config.server_port = 81;
  config.ctrl_port += 1;
  httpd_uri_t streamUri = {"/stream", HTTP_GET, streamHandler, nullptr};
  if (httpd_start(&streamHttpd, &config) == ESP_OK) {
    httpd_register_uri_handler(streamHttpd, &streamUri);
  }
}

static void connectWiFi() {
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // modem sleep adds latency and drops frames
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  pinMode(FLASH_LED_GPIO_NUM, OUTPUT);
  digitalWrite(FLASH_LED_GPIO_NUM, LOW);

  if (!initCamera()) {
    Serial.println("Restarting in 5 s");
    delay(5000);
    ESP.restart();
  }
  
  connectWiFi();
  startServers();

  String ip = WiFi.localIP().toString();
  Serial.printf("Camera ready. Stream: http://%s:81/stream\n", ip.c_str());
  Serial.printf("Laptop: uv run stream_server.py --source http://%s:81/stream\n", ip.c_str());
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi lost, reconnecting");
    WiFi.reconnect();
    delay(5000);
    return;
  }
  delay(1000);
}
