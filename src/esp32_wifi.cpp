// To flash this code, run:
// pio run -e esp32 -t upload

#include <Arduino.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <WiFi.h>

const char* WIFI_SSID = "ASEM Training";
const char* WIFI_PASS = "Class@Asem";
const char* ARM_API_BASE = "http://10.4.0.99";

const char* MQTT_HOST = "mqtt.favoriot.com";
const uint16_t MQTT_PORT = 1883;

HardwareSerial unoSerial(2);
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

const uint8_t UNO_RX_PIN = 16;
const uint8_t UNO_TX_PIN = 17;
const uint32_t UNO_BAUD = 9600;

String rpcTopic() {
  return String(DEVICE_ACCESS_TOKEN) + "/v2/rpc";
}

String streamTopic() {
  return String(DEVICE_ACCESS_TOKEN) + "/v2/streams";
}

bool favoriotConfigured() {
  return DEVICE_ACCESS_TOKEN[0] != '\0';
}

void publishFavoriot(const char* key, const char* value) {
  if (!mqtt.connected()) return;

  char payload[180];
  snprintf(payload, sizeof(payload),
           "{\"device_developer_id\":\"%s\",\"data\":{\"%s\":\"%s\"}}",
           DEVICE_DEVELOPER_ID, key, value);
  mqtt.publish(streamTopic().c_str(), payload);
}

void report(const char* message) {
  Serial.println(message);
  unoSerial.println(message);
  publishFavoriot("status", message);
}

bool runArmApi(const char* path) {
  if (WiFi.status() != WL_CONNECTED) return false;

  HTTPClient http;
  String url = String(ARM_API_BASE) + path;
  http.begin(url);
  int code = http.POST("");
  http.end();

  return code >= 200 && code < 300;
}

bool fetchRed() {
  return runArmApi("/api/run/red");
}

bool fetchBlue() {
  return runArmApi("/api/run/blue");
}

bool fetchYellow() {
  return runArmApi("/api/run/yellow");
}

void runColor(const char* color) {
  bool ok = false;
  if (strcmp(color, "red") == 0) ok = fetchRed();
  else if (strcmp(color, "blue") == 0) ok = fetchBlue();
  else if (strcmp(color, "yellow") == 0) ok = fetchYellow();
  else return;

  char result[24];
  snprintf(result, sizeof(result), "%s_%s", color, ok ? "ok" : "fail");
  publishFavoriot("arm", result);
}

void handleUnoCommand(const String& command) {
  if (command == "RUN_RED") {
    report(fetchRed() ? "ARM_RED_OK" : "ARM_RED_FAIL");
  } else if (command == "RUN_BLUE") {
    report(fetchBlue() ? "ARM_BLUE_OK" : "ARM_BLUE_FAIL");
  } else if (command == "RUN_YELLOW") {
    report(fetchYellow() ? "ARM_YELLOW_OK" : "ARM_YELLOW_FAIL");
  }
}

void handleFavoriotMessage(char* topic, byte* payload, unsigned int length) {
  String message((const char*)payload, length);
  message.toLowerCase();
  Serial.printf("Favoriot %s: %s\n", topic, message.c_str());

  if (message.indexOf("red") >= 0) runColor("red");
  else if (message.indexOf("blue") >= 0) runColor("blue");
  else if (message.indexOf("yellow") >= 0) runColor("yellow");
}

void readUnoCommands() {
  static String command;

  while (unoSerial.available()) {
    char c = unoSerial.read();
    if (c == '\n') {
      command.trim();
      if (command.length() > 0) handleUnoCommand(command);
      command = "";
    } else if (c != '\r') {
      command += c;
    }
  }
}

void connectWifi() {
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long startMs = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startMs < 20000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    String message = "WIFI_CONNECTED " + WiFi.localIP().toString();
    report(message.c_str());
  } else {
    report("WIFI_FAILED");
  }
}

void connectFavoriot() {
  static unsigned long lastAttemptMs = 0;

  if (!favoriotConfigured() || WiFi.status() != WL_CONNECTED || mqtt.connected()) return;
  if (millis() - lastAttemptMs < 3000) return;
  lastAttemptMs = millis();

  String clientId = "esp32-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  Serial.println("Connecting to Favoriot MQTT...");
  if (mqtt.connect(clientId.c_str(), DEVICE_ACCESS_TOKEN, DEVICE_ACCESS_TOKEN)) {
    mqtt.subscribe(rpcTopic().c_str());
    publishFavoriot("status", "online");
    Serial.println("Favoriot MQTT connected");
  } else {
    Serial.printf("Favoriot MQTT failed: %d\n", mqtt.state());
  }
}

void setup() {
  Serial.begin(115200);
  unoSerial.begin(UNO_BAUD, SERIAL_8N1, UNO_RX_PIN, UNO_TX_PIN);
  connectWifi();
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(handleFavoriotMessage);
}

void loop() {
  static unsigned long lastReportMs = 0;

  readUnoCommands();

  if (WiFi.status() != WL_CONNECTED) {
    report("WIFI_DISCONNECTED");
    connectWifi();
    lastReportMs = millis();
  }

  connectFavoriot();
  mqtt.loop();

  if (millis() - lastReportMs >= 5000) {
    lastReportMs = millis();
    String message = "WIFI_CONNECTED " + WiFi.localIP().toString();
    report(message.c_str());
  }
}
